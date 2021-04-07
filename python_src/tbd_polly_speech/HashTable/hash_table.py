#!/usr/bin/env python3

import sys
import rospkg
import os
import yaml
import hashlib

# class for key tuples to .txt file masterfile


class Item:
    speaker = ""
    text = ""

    def __init__(self, speaker, text):
        self.speaker = speaker
        self.text = text

# .txt file hashtable


class HashTable:

    _main_index_path: str  # Path to the main index.
    _index_obj: dict  # The local copy of the index.

    def __init__(self, lib_directory):

        self._main_index_path = os.path.join(lib_directory, 'mainIndex.yaml')

        # try to load the index, if not exist, create one
        try:
            with open(self._main_index_path) as f:
                self._index_obj = yaml.safe_load(f)
        except OSError as e:
            # this probably means the file doesn't exist.
            with open(self._main_index_path, 'w+') as f:
                self._index_obj = {'index': {}}
                yaml.dump(self._index_obj, f)

    def hashing(self, item: Item) -> str:
        return hashlib.md5((item.speaker + item.text).encode('utf-8')).hexdigest()

    def find(self, item: Item) -> str:
        hash_value = self.hashing(item)
        if hash_value in self._index_obj['index']:
            # double check if the items are equal just in case there is a conflict
            if item.text != self._index_obj['index'][hash_value]['text']:
                return None
            return f"{hash_value}.wav"
        return None

    def insert(self, item: Item) -> None:
        hash_value = self.hashing(item)

        # ignore if already exist
        if hash_value in self._index_obj['index'] and item.text == self._index_obj['index'][hash_value]['text']:
            return

        # add it to the index
        self._index_obj['index'][hash_value] = {
            'text': item.text,
        }

        # write it to file
        with open(self._main_index_path, 'w+') as f:
            yaml.dump(self._index_obj, f)

    def delete(self, item: Item) -> None:
        hash_value = self.hashing(item)

        if hash_value in self._index_obj['index'] and item.text == self._index_obj['index'][hash_value]['text']:
            del self._index_obj['index'][hash_value]
            # write it to file
            with open(self._main_index_path, 'w+') as f:
                yaml.dump(self._index_obj, f)

    # delete entry from table

    def delete(self, item):
        hash = self.hashing(itemHashTable)
        with open(self._main_index_path) as f:
            lines = f.readlines()

            # exclude given line and truncate file
            for line in lines:
                if line != ("(%s, %s): %i.wav\n" % (item.speaker, item.text, hash)):
                    f.write(line)
            f.truncate()
            f.close()
        return

    # return number of entries in file
    def numEntries(self):
        with open(self._main_index_path) as f:
            for i, j in enumerate(f):
                pass

    # delete entry from table
    def delete(self, item):
        hash = self.hashing(itemHashTable)
        with open(self._main_index_path) as f:
            lines = f.readlines()

            # exclude given line and truncate file
            for line in lines:
                if line != ("(%s, %s): %i.wav\n" % (item.speaker, item.text, hash)):
                    f.write(line)
            f.truncate()
            f.close()
        return

    # return number of entries in file
    def numEntries(self):
        with open(self._main_index_path) as f:
            for i, j in enumerate(f):
                pass
        return i+1
