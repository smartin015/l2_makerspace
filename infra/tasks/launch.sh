#!/bin/bash
docker-compose -f ../base/base-compose.yml -f docker-compose.yml -f docker-compose-api-token.yml up
