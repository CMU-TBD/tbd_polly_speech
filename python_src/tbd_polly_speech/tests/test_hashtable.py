
from tbd_polly_speech.HashTable import HashTable, Item

def test_hashing():
    table = HashTable("/tmp/")
    item = Item("Joanna", "sure, this is what your database said.")
    h1 = table.hashing(item)
    h2 = table.hashing(item)
    assert h1 == h2
    item2 = Item("Joanna", "sure, this is what your database said.")
    h2 = table.hashing(item2)
    assert h1 == h2

def test_insert_and_remove():
    table = HashTable("/tmp/")
    
    item = Item("Joanna", "sure, this is what your database said.")
    table.insert(item)
    rtn = table.find(item)
    assert rtn is not None

def test_stable_hasing():
    table = HashTable("/tmp/")
    item = Item("Joanna", "Hello Roboceptionist")
    assert table.hashing(item) == "297e58f6923f9cb1b44356091c78b57e"