FROM python:3.8.18-slim-bullseye
WORKDIR /robo
COPY control /robo/control
COPY requirements.txt /robo/requirements.txt
RUN pip config set global.index-url https://pypi.doubanio.com/simple \
    && pip install -r requirements.txt  \
    && pip install opencv-python-headless
    # && sed -i 's|security.debian.org/debian-security|mirrors.ustc.edu.cn/debian-security|g' /etc/apt/sources.list \  
WORKDIR /robo/control
CMD ["bash", "start.sh"]