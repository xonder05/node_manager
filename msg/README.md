### Message type codes

- 10 - run
- 20 - launch
- 50 - stop
- 90 - response

### Response codes

- 00 - ok
    - starting
    - 01 - node started succesfully
    - 02 - node already running
    - 03 - launch succesfull
    - 04 - parameter change succesfull
    - stoping
    - 05 - node stopped succesfully

- 10 - manager errors
- 20 - subprocess errors
    - 21 - process cannot be started because it is already running
    - 25 - process cannot be stopped because it is not running
    - 29 - internal error