ARG BASE_IMAGE="ubuntu"
ARG TAG="24.04"
FROM ${BASE_IMAGE}:${TAG}

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

WORKDIR /${USER_NAME}

RUN set -eux; \
    # Ensure the requested UID/GID is available. If something already exists with that ID
    # (e.g. the default "ubuntu" user), rename it to the requested USER_NAME.
    existing_group=$(getent group "${USER_GID}" | cut -d: -f1 || true); \
    if [ -n "$existing_group" ]; then \
        if [ "$existing_group" != "${USER_NAME}" ]; then \
            groupmod -n "${USER_NAME}" "$existing_group"; \
        fi; \
    else \
        groupadd "${USER_NAME}" --gid "${USER_GID}"; \
    fi; \
    existing_user=$(getent passwd "${USER_UID}" | cut -d: -f1 || true); \
    if [ -n "$existing_user" ]; then \
        if [ "$existing_user" != "${USER_NAME}" ]; then \
            usermod -l "${USER_NAME}" -d "/home/${USER_NAME}" -m "$existing_user"; \
        fi; \
    else \
        useradd -l -m "${USER_NAME}" -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash; \
    fi

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    git \
    openjdk-17-jre-headless \
    bash-completion \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Ensure Gradle uses a supported JDK (Ubuntu 24.04 defaults to Java 21, which
# Gradle 7.6 does not support). This is only needed for the Docker build.
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
ENV PATH="$JAVA_HOME/bin:$PATH"

COPY Tools/environment_install/install-prereqs-ubuntu.sh /${USER_NAME}/Tools/environment_install/
COPY Tools/completion /${USER_NAME}/Tools/completion/

# Set passwordless sudo for user to allow to use our scripts.
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

RUN chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME} \
    && chown -R ${USER_NAME}:${USER_NAME} /${USER_NAME}

USER ${USER_NAME}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/home/${USER_NAME}/.cache/pip,sharing=locked,uid=${USER_UID},gid=${USER_GID} \
    SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK \
    DO_AP_STM_ENV=$DO_AP_STM_ENV \
    AP_DOCKER_BUILD=1 \
    USER=${USER_NAME} \
    Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Rectify git perms issue that seems to crop up only on OSX
RUN git config --global --add safe.directory $PWD

# Check that local/bin are in PATH for pip --user installed package
RUN echo "if [ -d \"\$HOME/.local/bin\" ] ; then\nPATH=\"\$HOME/.local/bin:\$PATH\"\nfi" >> ~/.ardupilot_env

# Clone & install Micro-XRCE-DDS-Gen dependency
RUN git clone --recurse-submodules --depth 1 --branch v4.7.1 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git /home/${USER_NAME}/Micro-XRCE-DDS-Gen \
    && cd /home/${USER_NAME}/Micro-XRCE-DDS-Gen \
    && ./gradlew assemble \
    && export AP_ENV_LOC="/home/${USER_NAME}/.ardupilot_env" \
    && echo "export PATH=\$PATH:$PWD/scripts" >> $AP_ENV_LOC

# Create entrypoint as docker cannot do shell substitution correctly
RUN export ARDUPILOT_ENTRYPOINT="/home/${USER_NAME}/ardupilot_entrypoint.sh" \
    && echo "#!/bin/bash" > $ARDUPILOT_ENTRYPOINT \
    && echo "set -e" >> $ARDUPILOT_ENTRYPOINT \
    && echo "source /home/${USER_NAME}/.ardupilot_env" >> $ARDUPILOT_ENTRYPOINT \
    && echo 'exec "$@"' >> $ARDUPILOT_ENTRYPOINT \
    && chmod +x $ARDUPILOT_ENTRYPOINT \
    && sudo mv $ARDUPILOT_ENTRYPOINT /ardupilot_entrypoint.sh

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

ENV CCACHE_MAXSIZE=1G
ENTRYPOINT ["/ardupilot_entrypoint.sh"]
CMD ["bash"]
