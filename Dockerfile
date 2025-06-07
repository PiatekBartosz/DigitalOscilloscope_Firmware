# Use an ARG for the SDK version to easily update it
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# TODO: update to 4.x.x
ARG ZEPHYR_SDK_VERSION=0.17.1
ARG ZEPHYR_VERSION_TAG=v3.7.0

# Install common dependencies and Zephyr requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
    make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1 \
    python3-venv ca-certificates sudo vim git-lfs \
    # Dependencies for specific native_sim targets if needed (e.g., GUI)
    # libglib2.0-dev libpixman-1-dev
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Zephyr SDK
# SDK will be installed in /opt/zephyr-sdk
WORKDIR /tmp
RUN wget "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz" && \
    tar -xf "zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz" -C /opt && \
    mv "/opt/zephyr-sdk-${ZEPHYR_SDK_VERSION}" /opt/zephyr-sdk && \
    # Run the SDK setup script (important for toolchain registration)
    # -t all installs all available toolchains, you can specify specific ones too e.g., -t arm-zephyr-eabi,riscv64-zephyr-elf
    /opt/zephyr-sdk/setup.sh -t all && \
    rm "zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz"

# Install west (Zephyr's meta-tool)
RUN pip3 install --no-cache-dir west

# Create a non-root user for development
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# Switch to the non-root user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Set environment variables that west and Zephyr scripts will use
# These are also often set by zephyr-env.sh, but good to have them explicitly for clarity
ENV ZEPHYR_BASE=/opt/zephyr/zephyr
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# Initialize Zephyr project in /opt/zephyr
# Note: /opt is typically root-owned. If we want user to own /opt/zephyr,
# we could 'sudo mkdir /opt/zephyr && sudo chown zephyrdev:zephyrdev /opt/zephyr' before this.
# Or, initialize in the user's home directory.
# For this setup, we'll initialize in /opt/zephyr as requested, and
# the user will have write access because we chown it after west init.
RUN sudo mkdir -p /opt/zephyr && sudo chown ${USERNAME}:${USERNAME} /opt/zephyr
WORKDIR /opt/zephyr

# TODO: limit donwloader HAL
# Initialize Zephyr. Use --mr (manifest revision) to pin to a specific Zephyr version/tag/branch.
RUN west init . --mr ${ZEPHYR_VERSION_TAG}
# Fetch all modules and repositories
RUN west update
# Export Zephyr CMake package (makes Zephyr findable by CMake)
RUN west zephyr-export
# Install Python dependencies for Zephyr and its modules
RUN pip3 install --user --no-cache-dir -r ${ZEPHYR_BASE}/scripts/requirements.txt

# Set up .bashrc for the user
# This ensures the Zephyr environment is activated on login and west is in PATH
RUN echo 'export PATH=$HOME/.local/bin:$PATH' >> /home/${USERNAME}/.bashrc && \
    echo 'source /opt/zephyr/zephyr-env.sh' >> /home/${USERNAME}/.bashrc && \
    # The following are often set by zephyr-env.sh, but explicit ensures they are set.
    echo 'export ZEPHYR_TOOLCHAIN_VARIANT=zephyr' >> /home/${USERNAME}/.bashrc && \
    echo 'export ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk' >> /home/${USERNAME}/.bashrc

# Set a default working directory for when the container starts
WORKDIR /workspace
RUN mkdir -p /workspace && chown ${USERNAME}:${USERNAME} /workspace

# Default command to start a bash shell
CMD ["/bin/bash"]
