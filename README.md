# clover_SARC

Para rodar da mesma forma
- Colocar o arquivo 'Changes_To_Clover/sarc.world' em 'clover/clover_simulation/resources/worlds/'
- Colocar a pasta 'Changes_To_Clover/red/' em 'clover/clover_simulation/models/'
- Editar a linha 19 do arquivo 'clover/clover_simulation/launch/simulator.launch' colocando em value: "$(find clover_simulation)/resources/worlds/sarc.world"
- Rodar o arquivo teste.py
