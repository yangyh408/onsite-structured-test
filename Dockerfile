FROM yangyh408/onsite-structured-test:basic-image

ENV LANG C.UTF-8
RUN mkdir ./outputs
COPY planner ./planner
COPY requirements.txt ./
RUN python -m pip install -r requirements.txt -i https://mirrors.cloud.tencent.com/pypi/simple --no-cache-dir
VOLUME ["/outputs"]
RUN chmod -R 777 ./*
RUN cp -r /OnSite/TessNG/. /usr/lib
ENTRYPOINT [ "python", "/OnSite/main.py" ]

