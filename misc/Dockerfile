FROM ubuntu:xenial
RUN apt-get update -qq && \
    apt-get install -y software-properties-common wget && \
    apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" && \
    wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    apt-get update -qq && \
    apt-get install -y build-essential cmake clang-8 clang-tools-8 clang-tidy-8 && \
    rm -rf /var/lib/apt/lists/*
RUN ln -sf /usr/bin/clang-8 /usr/bin/clang && \
    ln -sf /usr/bin/clang++-8 /usr/bin/clang++ && \
    ln -sf /usr/bin/clang-tidy-8 /usr/bin/clang-tidy
