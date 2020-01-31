from node:12 
WORKDIR /usr/src/app

copy package*.json ./

RUN npm install

COPY tsconfig.json /usr/src/app
COPY src /usr/src/app/src

EXPOSE 5000
CMD ["npm", "run", "dev"]
