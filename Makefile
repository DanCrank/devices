.DEFAULT_GOAL := build

fmt:
	go fmt registers.go sx1231.go
	go fmt cmd/main.go
.PHONY:fmt

lint: fmt
	golint registers.go sx1231.go
	golint cmd/main.go
.PHONY:lint

vet: fmt
	go vet registers.go sx1231.go
	go vet cmd/main.go
.PHONY:vet

build: vet
	go build -o test cmd/main.go
.PHONY:build
