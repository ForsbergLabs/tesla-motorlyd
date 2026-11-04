# Tesla Motorlyd

Statisk nettapp for Tesla-nettleseren som lager motorlyd med Web Audio. GPS-farten styrer lyden: turtallet følger bilens fart, akselerasjon og giring. Du kan også bruke en farts-slider i testmodus, eller kjøre en simulert demo-tur uten GPS.

Ingen byggesteg. Hele siden er `index.html`, `favicon.ico` og `src/*.js`.

Publisert på GitHub Pages: https://forsberglabs.github.io/tesla-motorlyd/

Åpne lenken i bilens nettleser, trykk «Start motor», og velg «GPS / Kjøring» for å la lyden følge den ekte farten.

## Elbillyd: «Porsche Taycan-stil (EV)»

Et eget lydvalg inspirert av Porsches Electric Sport Sound, laget helt syntetisk i `src/electric.js` (ingen lydopptak). En dyp, futuristisk turbin-/romskip-dur der tonehøyden stiger jevnt med farten (ett utvekslingstrinn, ingen gir), et motorhvin som følger motorturtallet, og volum og lysstyrke som øker med gasspådraget. Ved slipp blir lyden mykere, og ved regenerering kommer en mørkere, synkende tone. Samme GPS-, test- og demo-styring som motorlydene.
