package arm

import (
	"bufio"
	"context"
	"net"
	"strings"
	"testing"
)

// runDashCmd drives a single dashboard command wrapper against an in-process
// fake server and returns the exact bytes the wrapper wrote to the socket
// (minus the trailing newline) plus the wrapper's error.
//
// net.Pipe gives a connected conn pair with no real network, and because this
// test is in-package it can set the unexported dashClient fields directly.
func runDashCmd(t *testing.T, reply string, call func(*dashClient) error) (string, error) {
	t.Helper()

	server, client := net.Pipe()
	defer server.Close()
	defer client.Close()

	c := &dashClient{conn: client}
	c.scanner = bufio.NewScanner(client)
	c.scanner.Split(splitOnSemicolon)

	// fake dashboard: read exactly one command line, capture it, then reply.
	got := make(chan string, 1)
	go func() {
		r := bufio.NewReader(server)
		line, _ := r.ReadString('\n')
		got <- strings.TrimRight(line, "\r\n")
		_, _ = server.Write([]byte(reply))
	}()

	err := call(c)
	return <-got, err
}

// TestDashWireStrings asserts the EXACT wire string each fixed command wrapper
// emits. This is the regression guard for the V4 protocol fixes: the previous
// tests never inspected the socket bytes, which is how the legacy (V3) command
// names escaped review.
func TestDashWireStrings(t *testing.T) {
	cases := []struct {
		name string
		call func(*dashClient) error
		want string
	}{
		{
			name: "velJ",
			call: func(c *dashClient) error { return c.velJ(context.Background(), 50) },
			want: "VelJ(50)",
		},
		{
			name: "accJ",
			call: func(c *dashClient) error { return c.accJ(context.Background(), 50) },
			want: "AccJ(50)",
		},
		{
			name: "jointMovJ",
			call: func(c *dashClient) error {
				return c.jointMovJ(context.Background(), [6]float64{10.5, -20.25, 30, 0, 0.1234, -0.0001})
			},
			want: "MovJ(joint={10.5000,-20.2500,30.0000,0.0000,0.1234,-0.0001})",
		},
		{
			name: "emergencyStop",
			call: func(c *dashClient) error { return c.emergencyStop(context.Background()) },
			want: "EmergencyStop(1)",
		},
		{
			name: "stop",
			call: func(c *dashClient) error { return c.stop(context.Background()) },
			want: "Stop()",
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got, err := runDashCmd(t, "0,{},ok;", tc.call)
			if err != nil {
				t.Fatalf("%s returned error: %v", tc.name, err)
			}
			if got != tc.want {
				t.Errorf("%s wrote %q, want %q", tc.name, got, tc.want)
			}
		})
	}
}
