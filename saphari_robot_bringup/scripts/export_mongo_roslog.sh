#!/bin/bash

mongodump --db Safe-Interaction_picking-surgical-instruments_0 --collection logged_designators --out logged_designators
mongodump --db Safe-Interaction_picking-surgical-instruments_0 --collection tf --out tf
