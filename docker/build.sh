cp -r ../audio ./audio

#docker build -t astroviz -f devel.Dockerfile .
docker build -t astroviz -f shelfy.Dockerfile .
