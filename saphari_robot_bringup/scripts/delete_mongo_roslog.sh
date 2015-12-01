#!/bin/bash

mongo roslog --eval 'db.dropDatabase()'
mongo roslog --eval 'db.logged_designators.ensureIndex( { "designator.OBJ._id" : 1 } )'
