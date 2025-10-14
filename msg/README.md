### Message type codes

- 10 - run
- 20 - launch
- 50 - stop
- 90 - response

### Response codes

- 00 - success
    - 01 - node started succesfully
    - 04 - parameter change succesfull
    - 05 - node stopped succesfully

- 10 - warnings
    - 11 node with same configuration is already running
    - 15 node cannot be stopped because it does not exist

- 20 - errors
    - 25 - process could not be stopped even with sigkill
    - 26 - waitpid error