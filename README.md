# Oakville Dynamics Rebuilt 2026 Robot

2026 FRC Source Code for Rebuilt

## Build Status

| Action   | Status                                                                                                                                                                                                            |
| -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CI       | [![Build](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/ci.yml)                            |
| Qodana   | [![Qodana](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/qodana.yml/badge.svg)](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/qodana.yml)                   |
| CodeQL   | [![CodeQL Scanning](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/codeql.yml/badge.svg)](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/codeql.yml)          |
| Spotless | [![Syntax Check](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/syntax-check.yml/badge.svg)](https://github.com/OakvilleDynamics/2026-Robot/actions/workflows/syntax-check.yml) |

## How to use

1. Clone repository
2. Open the project in your WPILib Visual Studio Code (VS Code) after cloning
3. Start hacking away!

## Features

- Preconfigured setup for GitHub Actions (helpful for [CI/CD](https://en.wikipedia.org/wiki/CI/CD))
  - Build action for building the robot code (helpful for ensuring code compiles)
  - Unit testing for ensuring your code does function as intended
  - [JetBrains Qodana](https://www.jetbrains.com/qodana/) action for static analysis (helpful for finding bugs and code smells)
  - [CodeQL](https://codeql.github.com/) action for static analysis and security scanning (helpful for finding bugs and security vulnerabilities)
  - [Spotless](https://github.com/diffplug/spotless) enforcement action for code formatting (helpful for keeping code cleanly formatted after commits)
  - [Gradle Validation](https://github.com/gradle/actions/blob/main/docs/wrapper-validation.md) action for validating the Gradle wrapper (helpful for ensuring [supply chain](https://en.wikipedia.org/wiki/Supply_chain_attack) security)
- Preconfigured setup for [Command-Based Robot](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) projects (helpful for getting started)
- [Dependabot](https://docs.github.com/en/code-security/dependabot) for dependency updates (helpful for keeping dependencies up to date)
- Preconfigured setup for [Spotless](https://github.com/diffplug/spotless) inside of Gradle (helpful for keeping code cleanly formatted during development)
  - Automatic formatting on every Gradle build is enabled by default
  - A syntax check if formatting was not applied locally will be performed on every push to `main`, including pull requests into `main`

## Requirements

- WPILib 2026.2.1
- Internet connection (for Gradle to download dependencies)
