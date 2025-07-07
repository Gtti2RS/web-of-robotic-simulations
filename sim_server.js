const { Servient, Helpers } = require("@node-wot/core");
const { HttpServer } = require("@node-wot/binding-http");
const fs = require("fs");

const servient = new Servient();
servient.addServer(new HttpServer({ port: 8888 }));

const td = JSON.parse(fs.readFileSync("sim.json", "utf-8"));

servient.start().then(async (WoT) => {
  const thingInstance = await WoT.produce(td);

  thingInstance.setActionHandler("start", async () => {
    console.log("Simulation started");
    return "Simulation started";
  });

  thingInstance.setActionHandler("pause", async () => {
    console.log("Simulation paused");
    return "Simulation paused";
  });

  thingInstance.setActionHandler("reset", async () => {
    console.log("Simulation reset");
    return "Simulation reset";
  });

thingInstance.setActionHandler("spawn", async (params) => {
  const input = await params.value(); // resolves WoT.Value
  const { name, type } = input;
  console.log("Spawning entity:", name, type);
  return `Spawned ${name} of type ${type}`;
});


  await thingInstance.expose();
  console.log(`TD available at: http://localhost:8888/${thingInstance.getThingDescription().title}`);
});
