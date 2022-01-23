.DEFAULT_GOAL := build

fmt:
	go fmt sx1231/registers.go sx1231/sx1231.go
	go fmt cmd/sx1231-test/main.go
.PHONY:fmt

lint: fmt
	golint sx1231/registers.go sx1231/sx1231.go
	golint cmd/sx1231-test/main.go
.PHONY:lint

vet: fmt
	go vet sx1231/registers.go sx1231/sx1231.go
	go vet cmd/sx1231-test/main.go
.PHONY:vet

build: vet
	go build -o test cmd/sx1231-test/main.go
.PHONY:build
