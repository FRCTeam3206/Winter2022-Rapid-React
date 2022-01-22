# 2022-Rapid-React

This repository contains the robot code for FRC Team 3206 "Royal T-Wrecks" for 2022 Rapid React.

## Using git and GitHub
*NOTE: If you are using the shared CREATE_01 computer, you are signed in to GitHub with the "RoyalT-Wrecks" account. If you are working from your own computer, please use your own GitHub account. If you have any trouble using git or GitHub, post a note on the team Discord.*

`git` is a popular version control system for managing source code. VCS provides a solution to the problem of managing code changes when multiple people are contributing to the same code base. It can also make it easier to keep track of changes and correct ones that cause bugs.

In a VCS system, all code changes are tracked and can be reverted if they cause problems. `git` does this using the concept of **branches** and **commits**. 

All repositories have at least one branch (called 'main' in this repo). When you want to modify the code, you create a new branch and edit the code in that branch. Once it has been tested, you then merge the new branch back into the main branch. `git` supports having multiple parallel branches, so it is possible to work on several differnt parts of the code base at the same time. A good branch is one that represents a single type of change to the code base, for example a branch to fix a specifc bug or one to add a new feature.

Code changes are added to a branch with a commit. This allows you to specify exactly which changed files are added to the branch and document what has been changed in a given commit. 

### Command Line Interface
The WPILib Documentation includes a [Git Version Control Introduction](https://docs.wpilib.org/en/stable/docs/software/basic-programming/git-getting-started.html) that provides a good overview of using the `git` command line from the `git bash` shell.

### Git and GitHub in VSCode
VS Code has built-in support for managing git repositories, which allows you to conduct the common actions directly fromm the IDE. We also have the GitHub extension installed so that you can create pull requests in the IDE.


## Coding Rules

* When working on code, always create a branch with a meaningful name followed by your initials (eg. `documentation_CRS`). 
* Commit frequently and push code to GitHub every day. This makes sure that everyone will have access to the latest version of any branch.   
* Do not commit directly to `main`. Create a `pull request` and the code will be reviewed and then merged if appropriate. This ensures that the `main` branch represents fully functional code.
* Before competition, create a new branch for that competition. All edits made during the competition should be made to that branch. After we return, we can review changes and decide which changes we want to merge back to `main`.

