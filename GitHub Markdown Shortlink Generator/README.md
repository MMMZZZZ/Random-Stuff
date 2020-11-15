# Shortlink Generator for GitHub Markdown Files

This Python script generates shortlinks to every header of a markdown file using GitHubs shortlink service [git.io](https://git.io). Resulting links can be printed to the console and/or stored in a json file.

## Requirements

* [Python >= 3.7](https://www.python.org/downloads/)
* [Python requests module](https://pypi.org/project/requests/)

## Usage

Open a console, navigate to the folder with the python script and type
```
python GithubMDShortlinkGen.py -h
```
It will display the help, explaining all the options.

### Example

Using 
```
python GithubMDShortlinkGen.py --onlinefile "https://github.com/MMMZZZZ/Random-Stuff/blob/master/GitHub%20Markdown%20Shortlink%20Generator/README.md"
```

You get
```
# Shortlink Generator for GitHub Markdown Files: https://git.io/JkZnM
## Requirements                                : https://git.io/JkZnD
## Usage                                       : https://git.io/JkZnS
### Example                                    : https://git.io/JkZn9
## License                                     : https://git.io/JkZnH

```

## License

As everything in this repository this script is licensed under MPL-2.0
