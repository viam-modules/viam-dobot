.PHONY: all module clean lint test

BIN     = bin/viam-cr10a
TARBALL = bin/module.tar.gz

GO_BUILD_ENV :=
GOFLAGS = -trimpath
LDFLAGS = -s -w

all: $(BIN)

$(BIN): Makefile main.go arm/*.go arm/*.json
	mkdir -p bin
	 GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GOFLAGS) -ldflags '$(LDFLAGS)' -o $(BIN) .

module: $(BIN)
	tar -czf $(TARBALL) $(BIN) meta.json arm/cr10a.urdf arm/meshes arm/3d_models

lint:
	go vet ./...
	gofmt -l . | (! grep .)

test:
	go test ./...

clean:
	rm -rf bin
