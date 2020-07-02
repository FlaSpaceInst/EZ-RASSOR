FROM ros:melodic

# Copy code repository and script that specifies test procedure
COPY . /ez-rassor
COPY .github/build/entrypoint.sh /entrypoint.sh

CMD ["/bin/sh", "/entrypoint.sh"]