Usage of setup scripts:

1. Run installAllDependencies script
2. Run all needed additional scripts

Note:
* If you already downloaded the git repo without git lfs installed, some large files are probably missing. Delete and clone the repo again after git lfs is installed and ready (which is done in installAllDependencies script)
* Git files are copied as e.g. ".gitconfig". Hidden files and folders have a "." in front of their name on linux. Enable "Show hidden files" in linux or use ls -a to see them.
* Please do not modify files in initFiles/ folder. They are meant to be templates and only be copied from this folder and THEN modified (as in initGit.sh)


For webserver and virtual environment:
* setupVirtualEnvs.sh


