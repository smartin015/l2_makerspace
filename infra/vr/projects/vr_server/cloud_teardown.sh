#!/bin/bash
gcloud config set project l2-making
gcloud config set compute/zone us-east4
kubectl delete service l2vr-server
gcloud container clusters delete l2vr-server
