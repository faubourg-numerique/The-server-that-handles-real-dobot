const { HttpServer } = require('@node-wot/binding-http');
const { Servient } = require('@node-wot/core');
const ROSLIB = require('roslib');
const fs = require('fs');


const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9091'  
});

ros.on('connection', () => {
  console.log('Connecté à ROS.');
});

ros.on('error', (error) => {
  console.error('Erreur de connexion à ROS :', error);
});

ros.on('close', () => {
  console.log('Connexion à ROS fermée.');
});


const orientation = new ROSLIB.Topic({
  ros: ros,
  name: '/target_r',
  messageType: 'std_msgs/msg/Float64'
});


const x_position = new ROSLIB.Topic({
  ros: ros,
  name: '/target_x',
  messageType: 'std_msgs/msg/Float64'
});

const y_position = new ROSLIB.Topic({
  ros: ros,
  name: '/target_y',
  messageType: 'std_msgs/msg/Float64'
});

const z_position = new ROSLIB.Topic({
  ros: ros,
  name: '/target_z',
  messageType: 'std_msgs/msg/Float64'
});


const servient = new Servient();
servient.addServer(new HttpServer({ port: 9090 }));

let TD;
try {
  TD = JSON.parse(fs.readFileSync('./RobotTD.json', 'utf8'));
} catch (error) {
  console.error('Erreur de lecture du fichier Thing Description :', error);
  process.exit(1);
}

servient.start().then(async (WoT) => {
  WoT.produce(TD).then(async (thing) => {
   
      thing.setActionHandler('move', async (params) => {
        const { x, y, z, r } = await params.value();
        const orientations = new ROSLIB.Message({ data: parseFloat(r) });
        const xposition = new ROSLIB.Message({ data: parseFloat(x) });
        const yposition = new ROSLIB.Message({ data: parseFloat(y) });
        const zposition = new ROSLIB.Message({ data: parseFloat(z) });

        x_position.publish(xposition);
        y_position.publish(yposition);
        z_position.publish(zposition);
        orientation.publish(orientations);
      });
      
      console.log("Thing exposée :", JSON.stringify(thing.getThingDescription(), null, 2));
      await thing.expose();
      console.log('Serveur WoT en cours d\'exécution sur http://192.168.1.190:8081...'); 
  }).catch(err => {
    console.error('Erreur lors de la production de la Thing :', err);
  });
}).catch(err => {
  console.error('Erreur lors du démarrage du Servient :', err);
});
