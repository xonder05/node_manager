### Message type codes

- 10 - run
- 20 - launch
- 30 - call script
- 40 - read file
- 41 - write file
- 50 - stop
- 90 - response

### Response codes

- 00 - success
    - 01 - node started successfully
    - 03 - script executed successfully
    - 04 - file read / write success
    - 05 - node stopped successfully

- 10 - warnings
    - 11 node with same configuration is already running
    - 15 node cannot be stopped because it does not exist

- 20 - errors
    - 23 - internal script error
    - 24 - could not read / write file
    - 25 - process could not be stopped even with sigkill
    - 26 - waitpid error