export default defineEventHandler(async (event) => {
  const res = event.node.res
  const req = event.node.req
  
  return event.context
})
