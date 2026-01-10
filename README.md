# Oakville Dynamics FRC Robot Template

A template for FRC robots using GradleRIO and WPILib

Prepackaged with GitHub Actions for CI/CD, Qodana for static analysis, CodeQL for static analysis and security scanning, Spotless for code formatting, and Gradle Validation for validating the Gradle wrapper

## Build Status

| Action   | Status                                                                                                                                                                                                            |
| -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CI       | [![Build](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/ci.yml/badge.svg)](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/ci.yml)                            |
| Qodana   | [![Qodana](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/qodana.yml/badge.svg)](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/qodana.yml)                   |
| CodeQL   | [![CodeQL Scanning](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/codeql.yml/badge.svg)](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/codeql.yml)          |
| Spotless | [![Syntax Check](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/syntax-check.yml/badge.svg)](https://github.com/OakvilleDynamics/frc-robot-template/actions/workflows/syntax-check.yml) |

## How to use

1. Create a template of this repository
2. Follow GitHub prompts for creating a repository
3. Clone newly created repository
4. Open the project in your WPILib Visual Studio Code (VS Code) after cloning
5. Start hacking away!

> [!NOTE]
>
> You should also change the `README.md` file to reflect your project for build statuses and other badges, otherwise the CI jobs listed here will point to the template
>
> Include other changes that are made to help users understand your project

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

> [!CAUTION]
>
> **Syntax Check**
>
> In the [`.github/workflows/syntax-check.yml`](.github/workflows/syntax-check.yml) workflow file, there is a [secret](https://docs.github.com/en/actions/security-guides/using-secrets-in-github-actions) that is used for automatically push changes from a GitHub Actions run, as by default the Actions runner itself does not have permission to push back to the repository. [This is intended behavior for security reasons under the normal `GITHUB_TOKEN` permission set](https://github.com/orgs/community/discussions/25702). To fix this, either:
>
> - Create a new [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens) to [scope the repository](https://docs.github.com/en/codespaces/managing-codespaces-for-your-organization/managing-development-environment-secrets-for-your-repository-or-organization#adding-secrets-for-a-repository) itself
> - Create a new [personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens) to [scope the organization](https://docs.github.com/en/codespaces/managing-codespaces-for-your-organization/managing-development-environment-secrets-for-your-repository-or-organization#adding-secrets-for-an-organization) to be used elsewhere (helpful if using this template on multiple different projects)
>
> If using a PAT in an organization, it is recommended to create a sock account that is generic but still has access to push to the repository, as this will be given [least privilege](https://en.wikipedia.org/wiki/Principle_of_least_privilege) to do other harm if that PAT were to be inadvertently public
>
> Make sure you also grant the sock account proper access to the repository, that includes branch protections and write access to the repository
>
> **JetBrains Qodana**
>
> In the [`.github/workflows/qodana.yml`](.github/workflows/qodana.yml) workflow file, there is a [secret](https://docs.github.com/en/actions/security-guides/using-secrets-in-github-actions) that is used to communicate to JetBrains Qodana Cloud. [This is required to be used with the workflow](https://www.jetbrains.com/help/qodana/github.html#Usage) as Qodana Cloud tracks all issues from all workflow runs for historical data. When left blank, the workflow will fail regardless of any issues. To fix this, either:
>
> - Create a new secret to [scope the repository](https://docs.github.com/en/actions/security-for-github-actions/security-guides/using-secrets-in-github-actions#creating-secrets-for-a-repository) for that project only
> - [Delete or disable the workflow](https://docs.github.com/en/actions/managing-workflow-runs-and-deployments/managing-workflow-runs/disabling-and-enabling-a-workflow#disabling-a-workflow) entirely to prevent CI failures.
>
> This means any time you create a repository from this template, you need to create a new project in Qodana Cloud and add the secret scoped to the repository.

- WPILib 2026.1.1
- Internet connection (for Gradle to download dependencies)
