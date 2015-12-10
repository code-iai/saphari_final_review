#!/bin/bash

mongo Safe-Interaction_picking-surgical-instruments_0 --eval 'db.dropDatabase()'
mongo Safe-Interaction_picking-surgical-instruments_0 --eval 'db.logged_designators.ensureIndex( { "designator.OBJ._id" : 1 } )'
mongo Safe-Interaction_picking-surgical-instruments_0 --eval 'db.logged_designators.ensureIndex( { "__recorded" : 1 } )'
mongo Safe-Interaction_picking-surgical-instruments_0 --eval 'db.tf.ensureIndex( { "__recorded" : 1 } )'
mongo Safe-Interaction_picking-surgical-instruments_0 --eval 'db.tf.ensureIndex( { "transforms.header.stamp" : 1 } )'
