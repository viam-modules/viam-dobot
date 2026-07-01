// Package main is the entrypoint for the Dobot CR10A Viam module.
//
// It registers two resource models for the arm API — `viam:dobot:cr10a` (the
// live controller driver) and `viam:dobot:cr10a-simulated` (a hardware-free
// simulated arm for testing motions without a connected robot) — and hands
// control to module.ModularMain, which handles the module gRPC handshake and
// lifecycle.
package main

import (
	dobot "github.com/viam-modules/viam-dobot/arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: dobot.Model},
		resource.APIModel{API: arm.API, Model: dobot.SimulatedModel},
	)
}
