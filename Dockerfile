FROM yangyh408/onsite-structured-test:basic-image

COPY planner ./planner
COPY requirements.txt ./

RUN pip install -r requirements.txt -i https://mirrors.cloud.tencent.com/pypi/simple --no-cache-dir

VOLUME ["/onsite-structured-test/TessNG/WorkSpace/Cert"]
VOLUME ["/onsite-structured-test/scenario"]
VOLUME ["/onsite-structured-test/config"]
VOLUME ["/onsite-structured-test/outputs"]

ENTRYPOINT [ "./run_test.sh" ]

