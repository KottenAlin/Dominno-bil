/*
 * Server-side plugin that connects to MQTT broker
 */

import mqtt from "mqtt"

export default defineNitroPlugin(nitroApp => {
  if(process.server) {
    const address = "10.22.4.8"
    const port = 1883

    const clientId = "domino-website"

    const client = mqtt.connect(`mqtt://${address}:${port}`, {
      clientId: clientId,
    })

    client.on("connect", () => {
      console.log("Client connected to MQTT broker")
    })

    client.on("error", (error) => {
      console.error("Connection error: ", error)
      client.end()  // This stops the client from reconnecting
    })

    console.log(client)

    nitroApp.client = "tjo"
    nitroApp.ctx.mqttClient = "hej"
  }
})
