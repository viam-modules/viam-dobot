// Package main is the entrypoint for the Dobot CR10A Viam module.
//
// It registers a single resource model — `viam-soleng:dobot:cr10a` for the
// arm API — and hands control to module.ModularMain, which handles the
// module gRPC handshake and lifecycle.
package main

import (
	dobot "github.com/viam-soleng/viam-dobot-cr10a/arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: dobot.Model},
	)
}
