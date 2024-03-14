<template>
  <div class="bg-red-500">
    <p>Text</p>
  </div>
</template>

<script setup>
const { $mqtt } = useNuxtApp()

$mqtt.publish("sko")
</script>
