/*
 * Client-side plugin to communicate with MQTT broker
 */

export default defineNuxtPlugin(nuxtApp => {
  const publish = async (json) => {
    return fetch("api/mqtt/publish", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(json),
    })
    .then(response => response.json())
  }

  nuxtApp.provide("mqtt", { publish });
})
