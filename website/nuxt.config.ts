// https://nuxt.com/docs/api/configuration/nuxt-config
export default defineNuxtConfig({
  devtools: {
    enabled: true
  },
  modules: [
    "@nuxtjs/tailwindcss"
  ],
  plugins: [
    "~/plugins/mqtt.ts"
  ],
  serverMiddleware: [
    "~/api/mqtt/publish.post.ts",
    "~/api/mqtt/publish.get.ts"
  ]
})
