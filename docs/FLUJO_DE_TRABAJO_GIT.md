# Flujo de trabajo con git

Reglas para que en cualquier momento se pueda contestar tres preguntas sin adivinar: **qué tiene grabado la placa, con qué parámetros, y qué cambió desde la última vez que anduvo.**

## 1. Ramas

- **`master` es siempre algo probado en el banco y grabable.** No se commitea en master nada que no se haya probado.
- Todo cambio va en una rama:
  - `prueba/<nombre>`: experimentos, ajustes, mediciones.
  - `arreglo/<nombre>`: corrección de un problema concreto.
  - `interfaz/<nombre>`: cambios solo de herramientas o interfaz.
- Una rama se mergea a master cuando la prueba dio bien. El mensaje del merge dice qué se probó y qué resultado dio.
- Una rama descartada se borra, o se deja con un último commit que diga por qué se descartó.
- Después de mergear, la rama se borra (`git branch -d prueba/<nombre>`).

## 2. Una sola carpeta de trabajo

- Se trabaja en `LAST DANCE/` y se cambia de rama con `git switch`. Nada de copias de la carpeta ni de `respaldos/`: git ya guarda cada versión.
- Si hace falta tener dos ramas abiertas a la vez, se usa `git worktree`, y se borra apenas se mergea (`git worktree remove`).
- No se intercambian archivos a mano (`maintemp.cpp`, `variantes_main/`). Una variante de firmware es una rama.

## 3. Nada queda sin commitear

- Al terminar el día, todo lo que se tocó queda commiteado en su rama y pusheado, aunque no esté terminado. Si no anda, el mensaje empieza con `WIP:`.
- Cambio sin commitear = cambio que se pierde, o que alguien graba sin saber que estaba.

## 4. Parámetros

- Cada cambio de `hw_config.h` va en **un commit propio**, con el valor viejo, el nuevo y el motivo:
  `hw_config: POS_KI 1500 -> 100 (ver docs/ELECCION_POS_KI.md)`
- Para probar ganancias no se edita `hw_config.h`: se ajustan en caliente con `M55`. Solo si se adoptan se commitean.
- Si cambia la tabla de parámetros, se actualiza la tabla de arriba de [CHANGELOG.md](../CHANGELOG.md).

## 5. Grabar la placa

- **Solo se graba con el árbol limpio** (`git status` sin cambios en `src/`). Así lo grabado es exactamente un commit.
- Se anota qué commit se grabó (`git log -1 --oneline`) en la descripción de la prueba o en la grabación de la interfaz.
- Después de grabar: `M56`, `M17`.

## 6. Versiones

- Cuando master pasa una prueba completa en el carril, se etiqueta:
  `git tag -a v2026-10-27 -m "Versión estable: <qué tiene>"` y `git push --tags`.
- Volver a una versión que andaba: `git switch --detach v2026-10-20`, grabar, y volver con `git switch master`.
- Ver qué cambió desde una versión: `git diff v2026-10-20 master -- src/`.

## 7. Sincronizar entre los dos

- Al empezar: `git pull`. Al terminar: `git push` (master y las ramas en curso).
- Cada uno commitea con su nombre. En la PC del laboratorio, que es compartida, se usa `--author`:
  `git commit --author="Tiago Meilan <...>"` o `git commit --author="Francisco Maresca <...>"`.
  Así `git log` dice quién hizo cada cosa (hoy casi todo figura como "μMachining").

## 8. Mensajes de commit

- Primera línea: qué cambió, en presente, menos de ~70 caracteres. Si toca un área concreta, empieza con ella (`Interfaz:`, `Calibración:`, `hw_config:`).
- Cuerpo: por qué, cómo se probó y qué dio. Con números.

## 9. Mediciones

- Las grabaciones y resultados de una prueba van a `mediciones/AAAA-MM-DD/` y se commitean en la misma rama que la prueba, junto con el commit del firmware que se usó.

## 10. CHANGELOG

- Cada merge a master agrega una entrada en [CHANGELOG.md](../CHANGELOG.md): fecha, commit, qué cambió, parámetros, cómo se validó y el `docs/` con el detalle.
- Lo que está en curso o abierto va en la sección "Sin versionar / en curso".

## Rutina diaria

```bash
git pull                              # 1. traer lo del otro
git switch -c prueba/<nombre>         # 2. rama para lo de hoy
# ... cambios, pruebas ...
git add <archivos> && git commit      # 3. commits chicos, con el resultado
git status                            # 4. limpio antes de grabar la placa
git push -u origin prueba/<nombre>    # 5. al terminar el día, aunque sea WIP
# si la prueba dio bien:
git switch master && git merge --no-ff prueba/<nombre>   # 6. merge + entrada en CHANGELOG
git push && git branch -d prueba/<nombre>
```
