#!/bin/bash

mongodump --db roslog --collection logged_designators --out logged_designators
mongodump --db roslog --collection tf --out tf
