export default defineEventHandler(async (event) => {
  const res = event.node.res
  const req = event.node.req
  
  var body = await readBody(event)

  console.log(body)

  return {hello: "world"}
})
