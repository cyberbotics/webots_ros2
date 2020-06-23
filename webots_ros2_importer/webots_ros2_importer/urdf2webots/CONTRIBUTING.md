# Contributing

We love pull requests from everyone.

## Getting Started: Create a Pull Request

1. Fork the repository: https://help.github.com/articles/fork-a-repo
2. Create a branch in your fork: https://help.github.com/articles/creating-and-deleting-branches-within-your-repository
3. Pull the branch as a pull request targeting `omichel:urdf2webots@master`: https://help.github.com/articles/creating-a-pull-request-from-a-fork
4. Wait for our awesome review :-)

## Coding Style

- Python should use the PEP8 standard (minus `E501`):

    > Note: using Atom with the `linter-flake8` linter package ensures that we respect our Python coding styles: `apm install linter-flake8`.

## Source Rules

- Avoid comitting files that exist elsewhere. Instead we should link to the source of these files.
- Avoid comitting files that can be re-created from other files using a Makefile, a script or a compiler.
