# Contributing to ropods-zephyr

Thank you for your interest in contributing to the ICM20948 SPI driver for Zephyr RTOS! We welcome contributions from the community.

## How to Contribute

### Reporting Issues

Before creating a new issue, please:
1. Search existing issues to avoid duplicates
2. Check if the issue is related to the ICM20948 driver specifically
3. Provide a clear description of the problem
4. Include steps to reproduce the issue
5. Mention your hardware setup and Zephyr version

### Submitting Pull Requests

1. **Fork the repository** and create your branch from `main`
2. **Follow coding standards** consistent with Zephyr project guidelines
3. **Test your changes** on relevant hardware platforms
4. **Update documentation** as needed
5. **Write clear commit messages** following conventional commit format

### Development Setup

1. Set up Zephyr development environment (v4.1.0+)
2. Clone this repository as an out-of-tree module
3. Build and test with the provided sample application

### Code Style

- Follow Zephyr coding style guidelines
- Use consistent indentation (tabs)
- Include appropriate header comments
- Document new functions and features

### Testing

- Test on nRF52 DK or similar supported hardware
- Verify both polling and interrupt modes work
- Check all sensor channels (accel, gyro, temperature)
- Test different sensor range configurations

## Questions?

Feel free to open an issue for questions about using or contributing to this project.

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.
