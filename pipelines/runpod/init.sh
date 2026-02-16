#!/bin/bash
#
# bash /workspace/bench/r2b_safety_lab/scripts/init.sh
# Current docker: runpod/pytorch:2.2.0-py3.10-cuda12.1.1-devel-ubuntu22.04

set -e

# Source configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/config.env" ]; then
    source "${SCRIPT_DIR}/config.env"
else
    echo "Error: config.env not found in ${SCRIPT_DIR}"
    exit 1
fi

log_info "Initializing R2B Safety Lab Setup..."

# Update system
apt-get update

if ! command -v htop &> /dev/null; then
    log_info "Installing htop..."
    apt-get install -y htop
fi

# SSH Configuration
log_info "Configuring SSH..."
mkdir -p "${SSH_DIR}"
if [ -f "${WORKSPACE_DIR}/.ssh/id_ed25519" ]; then
    cp "${WORKSPACE_DIR}/.ssh/id_ed25519" "${SSH_DIR}/id_ed25519"
    chmod 600 "${SSH_DIR}/id_ed25519"
    log_success "SSH key copied."
else
    log_warn "No SSH key found at ${WORKSPACE_DIR}/.ssh/id_ed25519. Git operations might fail."
fi

# Clone/Update Repository
log_info "Setting up repository..."
if [ ! -d "${REPO_DIR}" ]; then
    log_info "Cloning r2b_safety_lab..."
    git clone "${REPO_GIT_REF}" "${REPO_DIR}" --branch "${REPO_BRANCH}"
else
    log_info "Updating r2b_safety_lab..."
    cd "${REPO_DIR}"
    git pull
fi

# Virtual Environment Setup
log_info "Setting up Python environment..."
if [ ! -d "${VENV_DIR}" ]; then
    log_info "Creating .env venv..."
    python3 -m venv "${VENV_DIR}"
    # Upgrade pip inside the venv directly
    "${VENV_DIR}/bin/pip" install --upgrade pip
fi

# Activate venv for current session checks
source "${VENV_DIR}/bin/activate"

# Copy localrc for shell convenience
if [ -f "${SCRIPT_DIR}/.localrc" ]; then
    cp "${SCRIPT_DIR}/.localrc" ~/.localrc
    log_success "Updated ~/.localrc"
fi

log_success "Initialization complete! Ready to work."
log_info "Next: source ~/.localrc && bash ${SCRIPT_DIR}/build.sh"