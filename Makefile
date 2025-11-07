# Makefile for Robot Vision System
# Supports both AGX Orin and Server deployment

DOCKER_COMPOSE := docker compose
PROJECT_NAME := robot-vision-system
VERSION ?= latest

# User configuration
USER_ID := $(shell id -u)
GROUP_ID := $(shell id -g)
USERNAME := $(USER)

# Network configuration
ROS_DOMAIN_ID ?= 161
SERVER_IP ?= 10.28.121.28
ORIN_IP ?= 10.28.134.61
ROBOT_NAMESPACE ?=

# Detect architecture
ARCH := $(shell uname -m)
ifeq ($(ARCH),aarch64)
    MACHINE_TYPE := orin
    PROFILE := orin
else ifeq ($(ARCH),x86_64)
    MACHINE_TYPE := server
    PROFILE := server
else
    $(error Unsupported architecture: $(ARCH))
endif

# Export variables for docker-compose
export USER_ID
export GROUP_ID
export USERNAME
export ROS_DOMAIN_ID
export SERVER_IP
export ORIN_IP
export ROBOT_NAMESPACE
export VERSION

.DEFAULT_GOAL := help

.PHONY: help
help: ## Show this help message
	@echo "Robot Vision System - Docker Management"
	@echo ""
	@echo "Detected: ARCH=$(ARCH), MACHINE_TYPE=$(MACHINE_TYPE)"
	@echo ""
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Examples:"
	@echo "  make build-orin      # Build Orin image"
	@echo "  make build-server    # Build Server image"
	@echo "  make up              # Start services (auto-detect)"
	@echo "  make dev             # Start development container"
	@echo "  make logs            # View logs"
	@echo "  make clean           # Remove containers and volumes"

.PHONY: build
build: ## Build image for current architecture
	@echo "Building for $(MACHINE_TYPE)..."
	$(DOCKER_COMPOSE) --profile $(PROFILE) build

.PHONY: build-orin
build-orin: ## Build Orin client image
	$(DOCKER_COMPOSE) --profile orin build orin-client

.PHONY: build-server
build-server: ## Build server image
	$(DOCKER_COMPOSE) --profile server build server

.PHONY: build-dev
build-dev: ## Build development image
	$(DOCKER_COMPOSE) --profile dev build dev

.PHONY: build-all
build-all: ## Build all images
	$(DOCKER_COMPOSE) build

.PHONY: rebuild
rebuild: ## Rebuild image without cache
	$(DOCKER_COMPOSE) --profile $(PROFILE) build --no-cache

.PHONY: up
up: ## Start services for current architecture
	@echo "Starting $(MACHINE_TYPE) services..."
	$(DOCKER_COMPOSE) --profile $(PROFILE) up -d

.PHONY: up-orin
up-orin: ## Start Orin client
	$(DOCKER_COMPOSE) --profile orin up -d

.PHONY: up-server
up-server: ## Start server
	$(DOCKER_COMPOSE) --profile server up -d

.PHONY: up-all
up-all: ## Start all services
	$(DOCKER_COMPOSE) up -d

.PHONY: down
down: ## Stop all services
	$(DOCKER_COMPOSE) --profile orin --profile server --profile dev down

.PHONY: restart
restart: down up ## Restart services

.PHONY: dev
dev: ## Start development container
	$(DOCKER_COMPOSE) --profile dev run --rm dev

.PHONY: dev-orin
dev-orin: ## Start Orin development shell
	$(DOCKER_COMPOSE) --profile orin run --rm orin-client bash

.PHONY: dev-server
dev-server: ## Start Server development shell
	$(DOCKER_COMPOSE) --profile server run --rm server bash

.PHONY: shell
shell: ## Open shell in running container
ifeq ($(MACHINE_TYPE),orin)
	docker exec -it orin-client bash
else
	docker exec -it server bash
endif

.PHONY: rviz
rviz: ## Start RViz visualization
	@echo "Starting RViz..."
	@xhost +local:docker || true
	$(DOCKER_COMPOSE) --profile viz run --rm rviz

.PHONY: viz
viz: rviz ## Alias for rviz

.PHONY: logs
logs: ## Show logs from services
	$(DOCKER_COMPOSE) --profile $(PROFILE) logs -f

.PHONY: logs-orin
logs-orin: ## Show Orin logs
	$(DOCKER_COMPOSE) logs -f orin-client

.PHONY: logs-server
logs-server: ## Show Server logs
	$(DOCKER_COMPOSE) logs -f server

.PHONY: ps
ps: ## Show running containers
	$(DOCKER_COMPOSE) ps

.PHONY: status
status: ## Show system status
	@echo "=== Container Status ==="
	@$(DOCKER_COMPOSE) ps
	@echo ""
	@echo "=== Image List ==="
	@docker images | grep $(PROJECT_NAME) || echo "No images found"
	@echo ""
	@echo "=== Volume List ==="
	@docker volume ls | grep ros-data || echo "No volumes found"

.PHONY: stats
stats: ## Show container resource usage
	docker stats --no-stream

.PHONY: topic-list
topic-list: ## List ROS topics
	docker exec -it $(shell docker ps -q -f name=$(MACHINE_TYPE)) ros2 topic list

.PHONY: node-list
node-list: ## List ROS nodes
	docker exec -it $(shell docker ps -q -f name=$(MACHINE_TYPE)) ros2 node list

.PHONY: topic-hz
topic-hz: ## Check topic frequency (usage: make topic-hz TOPIC=/camera/color/image_raw)
	@if [ -z "$(TOPIC)" ]; then \
		echo "Error: Please specify TOPIC, e.g., make topic-hz TOPIC=/camera/color/image_raw"; \
		exit 1; \
	fi
	docker exec -it $(shell docker ps -q -f name=$(MACHINE_TYPE)) ros2 topic hz 

.PHONY: test-camera
test-camera: ## Test RealSense camera (Orin only)
	docker exec -it orin-client rs-enumerate-devices

.PHONY: test-multicast
test-multicast: ## Test DDS multicast
	docker exec -it $(shell docker ps -q -f name=$(MACHINE_TYPE)) ros2 multicast receive &
	sleep 2
	docker exec -it $(shell docker ps -q -f name=$(MACHINE_TYPE)) ros2 multicast send

.PHONY: test-network
test-network: ## Test network connectivity
	@echo "Testing network from $(MACHINE_TYPE)..."
ifeq ($(MACHINE_TYPE),orin)
	docker exec -it orin-client ping -c 3 $(SERVER_IP)
else
	docker exec -it server ping -c 3 $(ORIN_IP)
endif

.PHONY: clean
clean: down ## Remove containers and volumes
	@echo "Removing containers..."
	$(DOCKER_COMPOSE) --profile orin --profile server --profile dev rm -f
	@echo "Removing volumes..."
	docker volume rm -f orin-ros-data server-ros-data dev-ros-data || true

.PHONY: clean-all
clean-all: clean ## Remove everything including images
	@echo "Removing images..."
	docker images | grep $(PROJECT_NAME) | awk '{print $$3}' | xargs -r docker rmi -f

.PHONY: prune
prune: ## Prune Docker system
	docker system prune -af --volumes

.PHONY: setup-xhost
setup-xhost: ## Allow X11 forwarding for GUI
	xhost +local:docker

.PHONY: setup-udev
setup-udev: ## Setup udev rules for RealSense (requires sudo)
	@echo "Setting up udev rules for RealSense..."
	@if [ -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then \
		echo "udev rules already exist"; \
	else \
		echo "Creating udev rules..."; \
		echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules; \
		sudo udevadm control --reload-rules && sudo udevadm trigger; \
		echo "udev rules created. Please reconnect your RealSense camera."; \
	fi

.PHONY: setup
setup: setup-xhost setup-udev ## Run initial setup

.PHONY: config-network
config-network: ## Configure network settings
	@echo "Current network configuration:"
	@echo "  MACHINE_TYPE: $(MACHINE_TYPE)"
	@echo "  ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"
	@echo "  SERVER_IP: $(SERVER_IP)"
	@echo "  ORIN_IP: $(ORIN_IP)"
	@echo ""
	@echo "To change, edit .env file or set environment variables"

.PHONY: update-dds-config
update-dds-config: ## Update DDS configuration files
ifeq ($(MACHINE_TYPE),orin)
	@echo "Updating Orin DDS config..."
	@sed -i 's|<Peer address="[^"]*"/>|<Peer address="$(SERVER_IP)"/>|' src/client/bringup/config/dds/cyclonedds_orin.xml
	@echo "Updated peer to: $(SERVER_IP)"
else
	@echo "Updating Server DDS config..."
	@sed -i 's|<Peer address="[^"]*"/>|<Peer address="$(ORIN_IP)"/>|' src/server/bringup/config/dds/cyclonedds_server.xml
	@echo "Updated peer to: $(ORIN_IP)"
endif

.PHONY: lint
lint: ## Run linters
	@echo "Running linters..."
	@find src -name "*.py" -exec pylint {} + || true

.PHONY: format
format: ## Format Python code
	@echo "Formatting Python code..."
	@find src -name "*.py" -exec black {} +

.PHONY: info
info: ## Show system information
	@echo "=== System Information ==="
	@echo "Architecture: $(ARCH)"
	@echo "Machine Type: $(MACHINE_TYPE)"
	@echo "User: $(USERNAME) ($(USER_ID):$(GROUP_ID))"
	@echo ""
	@echo "=== Network Configuration ==="
	@echo "ROS Domain ID: $(ROS_DOMAIN_ID)"
	@echo "Server IP: $(SERVER_IP)"
	@echo "Orin IP: $(ORIN_IP)"
	@echo ""
	@echo "=== Docker Information ==="
	@docker version || echo "Docker not available"
	@echo ""
	@nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null || echo "No NVIDIA GPU detected"

.PHONY: version
version: ## Show version information
	@echo "Project: $(PROJECT_NAME)"
	@echo "Version: $(VERSION)"
	@echo "Machine Type: $(MACHINE_TYPE)"

.PHONY: quick-start
quick-start: build up logs ## Quick start (build + up + logs)

.PHONY: quick-start-orin
quick-start-orin: build-orin up-orin logs-orin ## Quick start Orin

.PHONY: quick-start-server
quick-start-server: build-server up-server logs-server ## Quick start Server