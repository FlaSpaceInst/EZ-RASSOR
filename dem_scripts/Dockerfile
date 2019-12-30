FROM ubuntu:18.04

RUN apt-get update && apt-get install -y \
    software-properties-common \
    python3 

RUN add-apt-repository -y ppa:ubuntugis/ppa

RUN apt-get update && apt-get install -y \ 
    gdal-bin \
    python-gdal \
    python3-gdal

COPY testGdal.py /tmp/
COPY NAC_DTM_APOLLO15.TIF /tmp/

CMD ["python3", "./tmp/testGdal.py", "./tmp/NAC_DTM_APOLLO15.TIF"]
