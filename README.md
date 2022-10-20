# Installation Instruction: Setting up the development environment and simulation

1. Run on a linux OS (Ubuntu/Debian/RHEL/Fedora/OpenSUSE/Arch/...)
1. Install [Docker Engine](https://docs.docker.com/engine/install/)
1. Install [Visual Studio Code](https://code.visualstudio.com/Download)
1. Install [GitHub CLI](https://github.com/cli/cli#installation)
1. Clone this repository `gh repo clone aeroteameindhoven/drone`
1. Open it in VSCode `code drone`
1. Install the [Remote Development Extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
1. Open the repository in the development container

## Inside the docker container

Ensure, before running these commands, that you see the following in the bottom
right of your screen:
![Devcontainer Example](.devcontainer/example.png)

and that your bash prompt starts with `dev@{somehexstring}`

### Useful tools

- `QGroundControl`
- `px4-gazebo`