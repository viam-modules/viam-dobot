.PHONY: all module clean lint test

BIN     = bin/viam-cr10a
TARBALL = bin/module.tar.gz

GOFLAGS = -trimpath
LDFLAGS = -s -w

all: $(BIN)

$(BIN):
	mkdir -p bin
	go build $(GOFLAGS) -ldflags '$(LDFLAGS)' -o $(BIN) .

module: $(BIN)
	tar -czf $(TARBALL) $(BIN) meta.json

lint:
	go vet ./...
	gofmt -l . | (! grep .)

test:
	go test ./...

clean:
	rm -rf bin
