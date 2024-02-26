FROM cyz713docker/onsite:basic-dev

ENV LANG C.UTF-8
RUN mkdir ./outputs
COPY planner ./planner
COPY requirements.txt ./
COPY TessNG/Cert/_cert ./TessNG/Cert
RUN python -m pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple --no-cache-dir
VOLUME ["/outputs"]
ENTRYPOINT ["./run.sh"]

