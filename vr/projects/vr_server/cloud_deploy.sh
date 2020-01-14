#!/bin/bash
gcloud config set project l2-making
gcloud config set compute/zone us-east4
gcloud container clusters create l2vr-server --num-nodes=1
kubectl create deployment l2vr-server --image=gcr.io/l2-making/vr-server
kubectl expose deployment l2vr-server --type=LoadBalancer --port 44444 --target-port 44444
kubectl get pods
kubectl get service
