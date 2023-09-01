struct Person
    myname::String
    age::Int64
end
  
struct Shoes
    shoesType::String
    colour::String
end
  
struct Student
    s::Person
    school::String
    shoes::Shoes
end
  
function printMyActivity(st::Student)
    println("I study at $(st.school) school")
end
  
struct Employee
    s::Person
    monthlyIncomes::Float64
    company::String
    shoes::Shoes
end

function printMyActivity(self::Employee)
    println("I work at $(self.company) company")
end
  
gymShoes = Shoes("gym","white")
proShoes = Shoes("classical","brown")

Marc = Student(Person("Marc",15),"Divine School",gymShoes)
MrBrown = Employee(Person("Brown",45),1200.0,"ABC Corporation Inc.", proShoes)

printMyActivity(Marc)
printMyActivity(MrBrown)