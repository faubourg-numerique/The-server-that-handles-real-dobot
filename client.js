const makeWoTinteraction = require('./classclient');
const TDfile = "http://localhost:9090/robot"; 
const client = new makeWoTinteraction(TDfile);

async function invokeMoveAction() {
    try {
        const moveParams = {
            x: 0,
            y: 0,
            z: 300,
            r: 0
        };
        await client.invokeAction('move', moveParams);
    } catch (error) {
        console.error('Erreur lors de l\'appel à l\'action move:', error);
    }
}

(async () => {
    await invokeMoveAction();
})();
