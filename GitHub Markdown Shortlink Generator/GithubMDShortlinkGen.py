import requests
import urllib.parse
import glob
from pathlib import Path
from typing import TextIO
import json
from multiprocessing import Pool
import argparse


def getShortUrl(url:str):
    files = {
        'url': (None, url),
    }

    test = requests.head(url)
    test.raise_for_status()
    response = requests.post('https://git.io/', files=files)

    return response.headers["location"]

def txt2tag(s:str):
    result = "#"
    s = s.lower()
    s = s.replace(" ", "-")
    for c in s:
        c: str
        if c.isascii():
            if c.isalnum() or c in "_-":
                result += c
        else:
            result += urllib.parse(c)

    return result

def getHeaders(fileStr:str):
    headers = list()
    for line in fileStr.splitlines():
        if not line.startswith("#"):
            continue

        header = dict()
        level = 0
        while line.startswith("#"):
            line = line[1:]
            level += 1
        header["title"] = line.lstrip("#").lstrip("\t ").rstrip("\r\n\t ")
        header["level"] = level
        header["tag"] = txt2tag(header["title"])
        headers.append(header)
    # Add "-1", "-2", ... for duplicate names
    headerTags = [h["tag"] for h in headers]
    for i, h in enumerate(reversed(headers)):
        occurences = headerTags.count(h["tag"]) - 1
        if occurences > 0:
            headers[-i - 1]["tag"] += "-" + str(occurences)

    return headers


if __name__ == '__main__':
    description = """GitHub Markdown Shortlink Creator by Max Zuidberg. Licensed under MPL-2.0
This script uses the GitHub shortlink service git.io to generate shortlinks to 
every header in the specified source file. 
Note: git.io will check if your link has already been shortened and if so, 
      returns the existing shortlink. In other words: it is safe to run 
      this script multiple times. 
"""
    parser = argparse.ArgumentParser(description="", formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--onlinefile",  metavar="link", type=str, required=True,  help="GitHub link to the markdown file. Will be used\r\nto generate the input links for git.io.")
    parser.add_argument("--offlinefile", metavar="file", type=str, required=False, help="Optional local path to the markdown file. If not\r\nspecified, the file specified with --onlinefile\r\nwill be downloaded and then parsed.")
    parser.add_argument("--json",        metavar="out",  type=str, required=False, help="Optional path for a JSON file that will contain\r\nthe titles and corresponding shortlinks.")
    parser.add_argument("--log",  default=2, type=int, required=False, help="Optional: Specify a different level of detail\r\nfor the console output. Default level: 2\r\n0: Errors only.\r\n1: Header title and shortlink (default)\r\n2: Header level (2 = \"##\"), title and shortlink\r\n3: Full json with title, level, tag, full and short link.")
    args = parser.parse_args()

    r = requests.head(args.onlinefile)

    if r.status_code != 200:
        parser.error("Link doesn't seem to be valid (Status code: " + str(r.status_code) + ").")

    fileContent: str

    if args.offlinefile:
        args.offlinefile = Path(args.offlinefile)
        if not args.offlinefile.exists():
            parser.error("Can't find specified offline file.")
        else:
            with open(args.offlinefile) as f:
                fileContent = f.read()
    else:
        rawLink = args.onlinefile.replace("blob", "raw")
        r = requests.get(rawLink)
        if r.status_code != 200:
            parser.error("Can't download raw file (Status code: " + str(r.status_code) + ").")
        else:
            fileContent = r.content.decode()
            if fileContent.lstrip("\r\n\t ").startswith("<!DOCTYPE html>"):
                parser.error("Downloaded file is no markdown file.")

    if args.log < 0 or args.log > 3:
        parser.error("Invalid log level: " + str(args.log))

    pool = Pool(processes=16)

    headers = getHeaders(fileContent)

    if len(headers) == 0:
        parser.error("Couldn't find any valid headers.")

    inputLinks = [args.onlinefile + h["tag"] for h in headers]
    for i,e in enumerate(inputLinks):
        headers[i]["full"] = e
    shortlinks = pool.imap(getShortUrl, inputLinks, 16)
    for i,e in enumerate(shortlinks):
        headers[i]["short"] = e

    maxLen = 0
    for h in headers:
        l = len(h["title"])
        if args.log == 2:
            l += h["level"] + 1
        if l > maxLen:
            maxLen = l

    if args.log:
        if args.log == 3:
            print(json.dumps(headers, indent=4))
        else:
           for h in headers:
                title = h["title"]
                if args.log == 2:
                    title = "#" * h["level"] + " " + title
                title += " " * (maxLen - len(title))
                print(title, ": ", h["short"], sep="")

    if args.json:
        args.json = Path(args.json)
        with open(args.json, "w") as f:
            json.dump(headers, f, indent=4)

