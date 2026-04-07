# Robotics Project Repository Guide

## Overview
This repository contains the source code for our 2025 robot, including all subsystems, vision processing, and motion planning components.

##Controller Bindings 
https://docs.google.com/spreadsheets/d/10Bk8kJeCZEpjuQmLOMcRfuR12O42bP0hZRbtEoB95dA/edit?usp=sharing

##Branching Strategy

To maintain a clean and efficient workflow, follow these guidelines for branch naming and merging:

### **Branch Naming Conventions**
- **Development Branches (`develop/`):** Used for experimental features, research, or algorithm testing.
  - Example: `develop/path-planning`, `develop/vision-improvements`
- **Hotfix Branches (`hotfix/`):** For urgent bug fixes that need to be patched immediately.
  - Example: `hotfix/encoder-calibration`, `hotfix/limelight-connection`
- **Release Branches (`release/`):** Used to prepare stable versions before merging into `main`.
  - Example: `release/1.0`, `release/2.1`

### **Merging Guidelines**
- **Feature and Development branches should be merged into `develop` after testing and review.**
- **Only fully tested and approved changes should be merged into `main`.**
- **Hotfixes can be merged directly into `main` if urgent, but must also be merged into `develop`.**
- **Release branches should be used to finalize versions before merging into `main`.**
- **Main should always be in a deployable state and never broken.**

## Development Workflow
1. **Create a branch based on the feature or experiment:**
   ```sh
   git checkout -b develop/coral-processor
   ```
2. **Commit and push changes:**
   ```sh
   git add .
   git commit -m "Added initial implementation of coral processor."
   git push origin feature/coral-processor
   ```
3. **When a stable set of features is ready, create a `release` branch from `develop`.**
   ```sh
   git checkout -b release/1.0 develop
   git push origin release/1.0
   ```
4. **Perform final testing and bug fixes in the `release` branch.**
5. **Once the `release` branch is stable, merge it into `main`.**
   ```sh
   git checkout main
   git merge release/1.0
   git push origin main
   ```
6. **Merge `release` back into `develop` to keep all branches up-to-date.**
   ```sh
   git checkout develop
   git merge release/1.0
   git push origin develop
   ```
7. **Delete the feature and release branches after merging.**
   ```sh
   git branch -d feature/coral-processor
   git branch -d release/1.0
   ```

## Commit Message Guidelines
- Use descriptive and concise commit messages.
- Format: `[Type]: Description`
  - Example: `feat: Added vision tracking for AprilTags`
  - Example: `fix: Corrected encoder offset calibration`

### **Commit Type Examples**
- `feat`: New feature implementation
- `fix`: Bug fix
- `refactor`: Code improvements without changing functionality
- `test`: Adding or modifying tests
- `docs`: Documentation updates
- `chore`: Maintenance or minor changes

By following this structure, we ensure a stable, working repository at all times.
a