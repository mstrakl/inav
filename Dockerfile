FROM ubuntu:jammy

ARG USER_ID
ARG GROUP_ID
ARG GDB

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y git cmake make ruby gcc g++ python3 python3-yaml ninja-build gcc-arm-none-eabi

RUN if [ "$GDB" = "yes" ]; then apt-get install -y gdb; fi

# Create group and user with specified IDs, handling conflicts
RUN if getent group $GROUP_ID >/dev/null 2>&1; then \
      groupmod -n inav $(getent group $GROUP_ID | cut -d: -f1); \
    else \
      groupadd --gid $GROUP_ID inav; \
    fi

RUN if getent passwd $USER_ID >/dev/null 2>&1; then \
      userdel -r $(getent passwd $USER_ID | cut -d: -f1); \
    fi && \
    useradd -m --uid $USER_ID --gid $GROUP_ID inav

USER inav

RUN git config --global --add safe.directory /src

WORKDIR /src/build

ENTRYPOINT ["/src/cmake/docker.sh"]
