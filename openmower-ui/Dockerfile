FROM debian:bookworm-slim AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV FLUTTER_HOME=/opt/flutter
ENV PATH="${FLUTTER_HOME}/bin:${FLUTTER_HOME}/bin/cache/dart-sdk/bin:${PATH}"

RUN apt-get update && apt-get install -y \
    curl git unzip xz-utils zip libglu1-mesa ca-certificates bash \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/flutter/flutter.git ${FLUTTER_HOME}
RUN flutter config --enable-web

WORKDIR /src
RUN git clone https://github.com/ClemensElflein/OpenMowerApp.git app

WORKDIR /src/app
RUN flutter pub get
RUN flutter build web --release

FROM nginx:alpine
COPY --from=builder /src/app/build/web /usr/share/nginx/html

RUN cat > /etc/nginx/conf.d/default.conf <<'NGINXEOF'
server {
  listen 9090;
  server_name _;
  root /usr/share/nginx/html;
  index index.html;

  location / {
    try_files $uri $uri/ /index.html;
  }
}
NGINXEOF

EXPOSE 9090
CMD ["nginx", "-g", "daemon off;"]
