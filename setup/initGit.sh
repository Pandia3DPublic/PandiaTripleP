#!/bin/bash
if [ $UID -eq 0 ] ;then
    echo "Aborting. Please run without sudo."
    exit
fi

user=$(logname)
echo "This script copies .gitconfig and .git-credentials to /home/$user/"
echo -n "Do you want to continue? (y/n) "
read answer
if [ "$answer" != "${answer#[Yy]}" ] ;then
    cp initFiles/gitconfig /home/$user/.gitconfig
    cp initFiles/git-credentials /home/$user/.git-credentials
    echo "Files successfully copied!"
    echo "Next you need to edit these files in /home/$user/ with your git user data."
    echo -n "Do you want to do this right away? (y/n) "
    read answer
    if [ "$answer" != "${answer#[Yy]}" ] ;then
        gedit /home/$user/.gitconfig
        gedit /home/$user/.git-credentials
    fi
    echo "Done"
else
    echo "Aborting"
    exit
fi

