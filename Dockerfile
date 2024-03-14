FROM yangyh408/onsite-structured-test:basic-image

COPY planner ./planner
COPY requirements.txt ./

RUN pip install -r requirements.txt -i https://mirrors.cloud.tencent.com/pypi/simple --no-cache-dir

VOLUME ["/onsite_structured_test/TessNG/WorkSpace/Cert"]
VOLUME ["/onsite_structured_test/scenario"]
VOLUME ["/onsite_structured_test/config"]
VOLUME ["/onsite_structured_test/outputs"]

ENTRYPOINT [ "./run_test.sh" ]

