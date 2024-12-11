const { Servient } = require('@node-wot/core');
const { HttpServer, HttpClientFactory, HttpsClientFactory } = require('@node-wot/binding-http');
const { Helpers } = require('@node-wot/core');

// Fonction utilitaire pour ajouter un délai
function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Classe pour gérer les interactions WoT
class MakeWoTInteraction {
    // Constructeur
    constructor(fileAddress, credentials) {
        this.TDfile = fileAddress; // Chemin local ou adresse IP distante de la description de la Thing
        this.client = new Servient(); // Initialise le serveur WoT
        this.client.addClientFactory(new HttpsClientFactory());
        this.client.addClientFactory(new HttpClientFactory());
        this.wotHelper = new Helpers(this.client); // Helper pour faciliter les interactions avec WoT

        // Ajoute des informations d'identification si nécessaire
        if (credentials) {
            this.client.addCredentials(credentials);
        }

        this.data = "null"; // Valeur par défaut pour les données
    }

    // Méthode pour invoquer une action sur la Thing
    async invokeAction(actionName, options) {
        try {
            const td = await this.wotHelper.fetch(this.TDfile); // Récupère la description de la Thing
            const WoT = await this.client.start();
            const thing = await WoT.consume(td);

            await thing.invokeAction(actionName, options); // Invoque l'action spécifiée
            console.log("Action : " + actionName);
             console.log("             ");
             console.log("-------------");
             console.log("             ");
        } catch (err) {
            console.error("Erreur lors de l'invocation de l'action :", err);
        }
    }

    // Méthode pour lire une propriété de la Thing
    async readProperty(propertyName) {
        try {
            const td = await this.wotHelper.fetch(this.TDfile);
            const WoT = await this.client.start();
            const thing = await WoT.consume(td);
            const data = (await thing.readProperty(propertyName)).value();

            return data; // Retourne la valeur de la propriété lue
        } catch (err) {
            console.error("Erreur lors de la lecture de la propriété :", err);
        }
    }

    // Méthode pour écrire une propriété sur la Thing
    async writeProperty(propertyName, options) {
        try {
            const td = await this.wotHelper.fetch(this.TDfile);
            const WoT = await this.client.start();
            const thing = await WoT.consume(td);

            await thing.writeProperty(propertyName, options); // Écrit la valeur spécifiée sur la propriété
            console.log("Propriété écrite : " + propertyName);
        } catch (err) {
            console.error("Erreur lors de l'écriture de la propriété :", err);
        }
    }

    // Méthode pour s'abonner à un événement sur la Thing
    async subscribeEvent(eventName, listener, errorListener, options) {
        try {
            const td = await this.wotHelper.fetch(this.TDfile);
            const WoT = await this.client.start();
            const thing = await WoT.consume(td);

            // S'abonne à l'événement spécifié
            const subscription = await thing.subscribeEvent(
                eventName,     // Nom de l'événement
                listener,      // Fonction de rappel pour gérer l'événement
                errorListener, // Fonction de rappel pour les erreurs (optionnel)
                options        // Options supplémentaires (optionnel)
            );

      
            return subscription; // Retourne l'objet d'abonnement pour un éventuel désabonnement
        } catch (err) {
            console.error("Erreur lors de l'abonnement à l'événement :", err);
        }
    }
}

module.exports = MakeWoTInteraction;

