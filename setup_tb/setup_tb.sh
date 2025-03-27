echo "Preparing Turtlebot for installation..."

sudo echo "Sudo permissions granted"

# Set color variables for messages
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

sudo apt update
# Ask if apt upgrade should be performed
printf "${YELLOW}Do you want to perform apt upgrade? (y/n)${NC}"
read -r apt_upgrade
if [ "$apt_upgrade" = "y" ]; then
    printf "${GREEN}apt upgrade started...\n${NC}"
    sudo apt upgrade
else
    printf "${GREEN}apt upgrade skipped\n${NC}"
fi

# Install basic programs: git, vim, tmux, htop, net-tools, openssh-server
sudo apt install git vim tmux htop net-tools openssh-server tmuxinator -y

# Check if Docker is installed
echo "The current setup uses Docker for seamless installation. Checking Docker..."
if ! command -v docker &> /dev/null
then
    echo -e "${RED}Docker is not installed. Do you want to install it? (y/n)${NC}"
    read -r install_docker
    if [ "$install_docker" = "y" ]; then
        printf "${GREEN}Installing Docker...\n${NC}"
        sudo apt-get update
        sudo apt-get install ca-certificates curl
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc

        echo \
            "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
            $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
            sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        sudo apt-get update
        sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
    else
        printf "${RED}Docker installation skipped\n${NC}"
    fi
else
    printf "${GREEN}Docker is already installed\n${NC}"
fi

# Ask the user to register the hostname and IP in the router for DHCP
printf "${YELLOW}Please, register the hostname in the router for DHCP\n"
printf "Hostname: $(hostname)\n"
printf "IP: $(hostname -I)\n"
printf "Press enter when done \n${NC}"
read -r

printf "${GREEN}There is an entrypoint file in the home directory for help :-)\n${NC}"
cp ${SCRIPT_PATH}/entrypoint.md ~/

# Ask if the user wants to change the ssh welcome message for the file ./install/ssh_welcome_message.md
printf "Do you want to change the ssh welcome message (default: y)? (y/n)"
read -r change_ssh_welcome_message
if [ "$change_ssh_welcome_message" = "n" ]; then
    printf "${YELLOW}SSH welcome message not changed \n${NC}"
else
    printf "${GREEN}Changing SSH welcome message... \n${NC}"
    sudo cp ${SCRIPT_PATH}/install/ssh_welcome_message.md /etc/motd
fi
