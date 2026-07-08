#!/bin/bash

# Add Python Raspberry Pi Remote Deployment Script
# Adds a new Pi as a Python deployment target to an existing Git project

set -e # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
  echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
  echo -e "${RED}[ERROR]${NC} $1"
}

# Function to test SSH connection
test_ssh_connection() {
  local user_host=$1
  print_status "Testing SSH connection to $user_host..."

  if ssh -o ConnectTimeout=10 -o BatchMode=yes "$user_host" exit 2>/dev/null; then
    print_success "SSH connection successful"
    return 0
  else
    print_error "SSH connection failed"
    echo "Please ensure:"
    echo "  1. Pi is powered on and connected to network"
    echo "  2. SSH is enabled on Pi"
    echo "  3. SSH keys are set up (run: ssh-copy-id $user_host)"
    exit 1
  fi
}

# Function to create Python post-receive hook
create_python_hook() {
  local project_name=$1
  local username=$2

  cat <<EOF
#!/bin/bash

source /home/$username/.local/bin/env

cd /var/www/$project_name

echo "=== Starting Python deployment at \$(date) ==="

# Deploy code
echo "Updating code files..."
git --git-dir=/var/git/$project_name.git --work-tree=/var/www/$project_name checkout -f

# Create/update virtual environment
uv venv --system-site-packages --clear
uv sync

# Activate virtual environment
echo "Activating virtual environment..."
source .venv/bin/activate

# Make Python scripts executable
find . -name "*.py" -exec chmod +x {} \; 2>/dev/null || true

# --- Screen session management ---
SESSION="$project_name"

# Stop previous screen session for this project, if any
if screen -ls | grep -q "\\.\${SESSION}\\b"; then
  echo "Stopping previous screen session: \$SESSION"
  screen -S "\$SESSION" -X quit
  sleep 0.5
fi

# Start new detached screen session running main.py.
# - python -u: unbuffered output so logs appear live in the scrollback
# - exec bash at the end: keeps the session alive after main.py exits
#   so you can attach and inspect output later
# - -h 10000: keep 10k lines of scrollback
echo "Starting screen session: \$SESSION"
screen -h 10000 -dmS "\$SESSION" bash -c \\
  "cd /var/www/$project_name && .venv/bin/python -u main.py; ec=\\\$?; echo; echo \"[main.py exited with code \\\$ec — session kept alive for inspection]\"; exec bash"

echo ""
echo "=== Python deployment completed successfully at \$(date) ==="
echo "Virtual environment: \$(which python)"
echo ""
echo "Screen session:  \$SESSION"
echo "  attach:        screen -r \$SESSION"
echo "  list sessions: screen -ls"
echo "  detach inside: Ctrl-A then D"
echo "  kill:          screen -S \$SESSION -X quit"
EOF
}

# Function to setup Pi via SSH
setup_pi() {
  local user_host=$1
  local project_name=$2
  local username=$3

  print_status "Configuring Pi for Python deployment..."

  # Create the setup script
  local setup_script=$(
    cat <<EOF
#!/bin/bash
set -e

echo "Setting up Pi for Python deployment: $project_name"

# Update package list
echo "Updating package list..."
sudo apt update

# Install Python dependencies
echo "Installing Python and Git..."
sudo apt install -y git python3-venv python3-pip python3-dev build-essential screen

# Install common Python system dependencies
echo "Installing Python system dependencies..."
sudo apt install -y libffi-dev libssl-dev || echo "Optional packages skipped"

# --- Add project-specific apt packages below ---
# e.g. for picamera2: sudo apt install -y python3-picamera2 --no-install-recommends
# e.g. for opencv:    sudo apt install -y libopencv-dev

# Installing uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source /home/$username/.local/bin/env

# Create directory structure
echo "Creating directories..."
sudo mkdir -p /var/git/$project_name.git
sudo mkdir -p /var/www/$project_name

# Initialize bare repository
echo "Initializing bare Git repository..."
cd /var/git/$project_name.git
sudo git init --bare

# Set ownership and permissions
echo "Setting permissions..."
sudo chown -R $username:$username /var/git/$project_name.git
sudo chown -R $username:$username /var/www/$project_name
sudo chmod -R 755 /var/git/$project_name.git
sudo chmod -R 755 /var/www/$project_name

echo "Pi Python setup completed!"
EOF
  )

  # Execute setup script on Pi
  echo "$setup_script" | ssh "$user_host" "bash"

  # Create and upload post-receive hook
  print_status "Installing Python deployment hook..."
  local hook_content=$(create_python_hook "$project_name" "$username")

  echo "$hook_content" | ssh "$user_host" "cat > /var/git/$project_name.git/hooks/post-receive"
  ssh "$user_host" "chmod +x /var/git/$project_name.git/hooks/post-receive"

  print_success "Pi configuration completed!"
}

# Function to add Git remote
add_git_remote() {
  local user_host=$1
  local project_name=$2
  local remote_name=$3

  print_status "Adding Git remote '$remote_name'..."

  # Remove existing remote if it exists
  if git remote get-url "$remote_name" >/dev/null 2>&1; then
    print_warning "Remote '$remote_name' already exists, updating..."
    git remote set-url "$remote_name" "$user_host:/var/git/$project_name.git"
  else
    git remote add "$remote_name" "$user_host:/var/git/$project_name.git"
  fi

  print_success "Git remote '$remote_name' configured"
}

# Function to test deployment
test_deployment() {
  local remote_name=$1
  local user_host=$2
  local project_name=$3

  print_status "Testing Python deployment..."

  # Get current branch
  local current_branch=$(git branch --show-current)

  # Push to Pi
  if git push "$remote_name" "$current_branch"; then
    print_success "Deployment successful!"

    # Verify Python environment on Pi
    print_status "Verifying Python environment on Pi..."
    ssh "$user_host" "cd /var/www/$project_name && ls -la"

    echo ""
    print_success "Pi remote '$remote_name' is ready for Python deployment!"
    echo ""
    echo -e "${GREEN}Deploy with:${NC} ${BLUE}git push $remote_name $current_branch${NC}"

  else
    print_error "Deployment test failed!"
    echo "Check the output above for errors."
    return 1
  fi
}

# Function to check Python project requirements
check_python_project() {
  print_status "Checking Python project setup..."

  if [ -f "pyproject.toml" ]; then
    print_success "Found pyproject.toml"
  else
    print_warning "No pyproject.toml found — uv sync on the Pi will have nothing to install"
  fi

  local py_files=$(find . -maxdepth 2 -name "*.py" -not -path "./.venv/*" | head -3)
  if [ -n "$py_files" ]; then
    print_success "Found Python files:"
    echo "$py_files" | sed 's/^/    /'
  else
    print_warning "No Python files found in current directory"
  fi

  echo ""
}

# Main function
main() {
  echo -e "${BLUE}"
  echo "========================================"
  echo "   Add Python Pi Remote Deployment"
  echo "========================================"
  echo -e "${NC}"
  echo ""

  # Verify we're in a Git repository
  if ! git rev-parse --git-dir >/dev/null 2>&1; then
    print_error "Not in a Git repository!"
    echo "Please run this script from your project's root directory."
    exit 1
  fi

  # Get current project info
  local default_project_name=$(basename "$(pwd)")
  echo "Current directory: $(pwd)"

  # Check Python project setup
  check_python_project

  # Get Pi information
  read -p "Pi IP address or hostname: " pi_host
  if [ -z "$pi_host" ]; then
    print_error "Pi host is required"
    exit 1
  fi

  echo ""
  read -p "Username on Pi [pi]: " username
  username=${username:-pi}

  # Test SSH connection
  test_ssh_connection "$username@$pi_host"

  # Get project information
  echo ""
  read -p "Project name [$default_project_name]: " project_name
  project_name=${project_name:-$default_project_name}

  # Validate project name
  if ! [[ "$project_name" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    print_error "Project name can only contain letters, numbers, underscores, and hyphens"
    exit 1
  fi

  # Get remote name
  echo ""
  read -p "Git remote name [pi]: " remote_name
  remote_name=${remote_name:-pi}

  local user_host="$username@$pi_host"

  # Summary
  echo ""
  echo -e "${YELLOW}Configuration Summary:${NC}"
  echo "  Pi Host: $pi_host"
  echo "  Username: $username"
  echo "  Project: $project_name (Python)"
  echo "  Remote name: $remote_name"
  echo ""

  read -p "Proceed with Python setup? (Y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "Setup cancelled."
    exit 0
  fi

  # Execute setup
  setup_pi "$user_host" "$project_name" "$username"
  add_git_remote "$user_host" "$project_name" "$remote_name"

  # Test deployment if we have commits
  echo ""
  if git log --oneline -1 >/dev/null 2>&1; then
    read -p "Test Python deployment now? (Y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
      test_deployment "$remote_name" "$user_host" "$project_name"
    else
      echo ""
      echo -e "${GREEN}Setup completed!${NC}"
      echo -e "Deploy when ready with: ${BLUE}git push $remote_name $(git branch --show-current)${NC}"
    fi
  else
    echo ""
    print_warning "No commits found in repository"
    echo "Commit your code first, then deploy with:"
    echo -e "  ${BLUE}git add .${NC}"
    echo -e "  ${BLUE}git commit -m \"Initial commit\"${NC}"
    echo -e "  ${BLUE}git push $remote_name main${NC}"
  fi

  echo ""
  echo -e "${GREEN}Python Deployment Commands:${NC}"
  echo -e "  Deploy:      ${BLUE}git push $remote_name <branch>${NC}"
  echo -e "  SSH to Pi:   ${BLUE}ssh $user_host${NC}"
  echo -e "  Check files: ${BLUE}ssh $user_host \"ls -la /var/www/$project_name\"${NC}"
  echo -e "  Run script:  ${BLUE}ssh $user_host \"cd /var/www/$project_name && .venv/bin/python main.py\"${NC}"
}

# Run main function
main "$@"
