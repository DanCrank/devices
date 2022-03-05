.DEFAULT_GOAL := build

fmt:
	go fmt registers.go sx1231.go
.PHONY:fmt

lint: fmt
	golint registers.go sx1231.go
.PHONY:lint

vet: fmt
	go vet registers.go sx1231.go
.PHONY:vet

build: vet
.PHONY:build
