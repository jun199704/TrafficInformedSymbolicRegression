module PopulationModule

using StatsBase: StatsBase
using DispatchDoctor: @unstable
using DynamicExpressions: AbstractExpression, string_tree, eval_tree_array
using ..CoreModule: AbstractOptions, Options, Dataset, RecordType, DATA_TYPE, LOSS_TYPE
using ..ComplexityModule: compute_complexity
using ..LossFunctionsModule: score_func, update_baseline_loss!
using ..AdaptiveParsimonyModule: RunningSearchStatistics
using ..MutationFunctionsModule: gen_random_tree
using ..PopMemberModule: PopMember
using ..UtilsModule: bottomk_fast, argmin_fast, PerThreadCache

using ..LossFunctionsModule: eval_tree_dispatch
using JSON
using PythonCall

# A list of members of the population, with easy constructors,
#  which allow for random generation of new populations
struct Population{T<:DATA_TYPE,L<:LOSS_TYPE,N<:AbstractExpression{T}}
    members::Array{PopMember{T,L,N},1}
    n::Int
end
"""
    Population(pop::Array{PopMember{T,L}, 1})

Create population from list of PopMembers.
"""
function Population(pop::Vector{<:PopMember})
    return Population(pop, size(pop, 1))
end

"""
    Population(dataset::Dataset{T,L};
               population_size, nlength::Int=3, options::AbstractOptions,
               nfeatures::Int)

Create random population and score them on the dataset.
"""
function Population(
    dataset::Dataset{T,L};
    options::AbstractOptions,
    population_size=nothing,
    nlength::Int=3,
    nfeatures::Int,
    npop=nothing,
) where {T,L}
    @assert (population_size !== nothing) ⊻ (npop !== nothing)
    population_size = if npop === nothing
        population_size
    else
        npop
    end
    return Population(
        [
            PopMember(
                dataset,
                gen_random_tree(nlength, options, nfeatures, T),
                options;
                parent=-1,
                deterministic=options.deterministic,
            ) for _ in 1:population_size
        ],
        population_size,
    )
end
"""
    Population(X::AbstractMatrix{T}, y::AbstractVector{T};
               population_size, nlength::Int=3,
               options::AbstractOptions, nfeatures::Int,
               loss_type::Type=Nothing)

Create random population and score them on the dataset.
"""
@unstable function Population(
    X::AbstractMatrix{T},
    y::AbstractVector{T};
    population_size=nothing,
    nlength::Int=3,
    options::AbstractOptions,
    nfeatures::Int,
    loss_type::Type{L}=Nothing,
    npop=nothing,
) where {T<:DATA_TYPE,L}
    @assert (population_size !== nothing) ⊻ (npop !== nothing)
    population_size = if npop === nothing
        population_size
    else
        npop
    end
    dataset = Dataset(X, y, L)
    update_baseline_loss!(dataset, options)
    return Population(
        dataset; population_size=population_size, options=options, nfeatures=nfeatures
    )
end

function Base.copy(pop::P)::P where {T,L,N,P<:Population{T,L,N}}
    copied_members = Vector{PopMember{T,L,N}}(undef, pop.n)
    Threads.@threads for i in 1:(pop.n)
        copied_members[i] = copy(pop.members[i])
    end
    return Population(copied_members)
end

# Sample random members of the population, and make a new one
function sample_pop(pop::P, options::AbstractOptions)::P where {P<:Population}
    return Population(
        StatsBase.sample(pop.members, options.tournament_selection_n; replace=false)
    )
end

# Sample the population, and get the best member from that sample
function best_of_sample(
    pop::Population{T,L,N},
    running_search_statistics::RunningSearchStatistics,
    options::AbstractOptions,
) where {T,L,N}
    sample = sample_pop(pop, options)
    return copy(_best_of_sample(sample.members, running_search_statistics, options))
end
function _best_of_sample(
    members::Vector{P},
    running_search_statistics::RunningSearchStatistics,
    options::AbstractOptions,
) where {T,L,P<:PopMember{T,L}}
    p = options.tournament_selection_p
    n = length(members)  # == tournament_selection_n
    scores = Vector{L}(undef, n)
    if options.use_frequency_in_tournament
        # Score based on frequency of that size occurring.
        # In the end, all sizes should be just as common in the population.
        adaptive_parsimony_scaling = L(options.adaptive_parsimony_scaling)
        # e.g., for 100% occupied at one size, exp(-20*1) = 2.061153622438558e-9; which seems like a good punishment for dominating the population.

        for i in 1:n
            member = members[i]
            size = compute_complexity(member, options)
            frequency = if (0 < size <= options.maxsize)
                L(running_search_statistics.normalized_frequencies[size])
            else
                L(0)
            end
            scores[i] = member.score * exp(adaptive_parsimony_scaling * frequency)
        end
    else
        map!(_get_score, scores, members)
    end

    chosen_idx = if p == 1.0
        argmin_fast(scores)
    else
        # First, decide what place we take (usually 1st place wins):
        tournament_winner = StatsBase.sample(get_tournament_selection_weights(options))
        # Then, find the member that won that place, given
        # their fitness:
        if tournament_winner == 1
            argmin_fast(scores)
        else
            bottomk_fast(scores, tournament_winner)[2][end]
        end
    end
    return members[chosen_idx]
end
_get_score(member::PopMember) = member.score

const CACHED_WEIGHTS =
    let init_k = collect(0:5),
        init_prob_each = 0.5f0 * (1 - 0.5f0) .^ init_k,
        test_weights = StatsBase.Weights(init_prob_each, sum(init_prob_each))

        PerThreadCache{Dict{Tuple{Int,Float32},typeof(test_weights)}}()
    end

@unstable function get_tournament_selection_weights(@nospecialize(options::AbstractOptions))
    n = options.tournament_selection_n::Int
    p = options.tournament_selection_p::Float32
    # Computing the weights for the tournament becomes quite expensive,
    return get!(CACHED_WEIGHTS, (n, p)) do
        k = collect(0:(n - 1))
        prob_each = p * ((1 - p) .^ k)

        return StatsBase.Weights(prob_each, sum(prob_each))
    end
end

function finalize_scores(
    dataset::Dataset{T,L}, pop::P, options::AbstractOptions
)::Tuple{P,Float64} where {T,L,P<:Population{T,L}}
    need_recalculate = options.batching
    num_evals = 0.0
    if need_recalculate
        for member in 1:(pop.n)
            score, loss = score_func(dataset, pop.members[member], options)
            pop.members[member].score = score
            pop.members[member].loss = loss
        end
        num_evals += pop.n
    end
    return (pop, num_evals)
end

function construct_datasets_simple(
    X,
    y
) 
    nout = size(y, 1)
    return [
        Dataset(
            X,
            y[j, :];
        ) for j in 1:nout
    ]
end

filename="/home/drivesim/Documents/SR/MSFTrainData/TrainingDatasetSimple.json"
traj_dict_total = open(filename, "r") do file
    JSON.parse(file)
end
filename="/home/drivesim/Documents/SR/MSFTrainData/TrainingDataset.json"
vehTrajsTotal = open(filename, "r") do file
    JSON.parse(file)
end
filename="/home/drivesim/Documents/SR/MSFTrainData/TrainingID.json"
trainingID = open(filename,"r") do file
    JSON.parse(file)
end

# Return best 10 examples
function best_sub_pop(pop::P, dataset::Dataset{T,L}, options::Options; topn::Int=20)::P where {T,L,P<:Population{T,L}}
    
    #best_sub_pop(pop::P; topn::Int=10)::P where {P<:Population}
    #best_idx = sortperm([pop.members[member].score for member in 1:(pop.n)])
    #println("selecting best population: Finally worked")
    #return Population(pop.members[best_idx[1:topn]])
    GC.gc()
    best_idx = sortperm([pop.members[member].score for member in 1:(pop.n)])
    #return Population(pop.members[best_idx[1:10]])
    #=
    loss_list = []
    for idx in best_idx[1:min(length(best_idx),topn)]
        push!(loss_list, pop.members[idx].loss)
    end
    min_loss = minimum(loss_list)
    println("minimum loss: ",min_loss)
    max_loss = maximum(loss_list)
    println("maximum loss: ",max_loss)

    complexity_list = []
    for idx in best_idx[1:min(length(best_idx),topn)]
        push!(complexity_list, compute_complexity(pop.members[idx], options))
    end
    println("complexty list: ", complexity_list)
    println("minimum complexty: ", minimum(complexity_list))
    println("maximum complexity: ", maximum(complexity_list))
    =#
    
    candidate_best_idx = []
    for idx in best_idx[1:min(length(best_idx), topn)]
        loss = pop.members[idx].loss
        complexity = compute_complexity(pop.members[idx], options)
        
        # Check the conditions and remove idx if needed
        if loss < 0.1 && complexity >7
            push!(candidate_best_idx,idx)
        end
    end
    println("best idx: ",best_idx[1:min(length(best_idx),topn)])
    
    println("candidate best idx: ",candidate_best_idx)
    println("candidate best idx length: ",length(candidate_best_idx))


    #println(trainingID)
    if length(candidate_best_idx)==0
        return Population(pop.members[best_idx[1:10]])
    else
        if length(candidate_best_idx)<=10
            return Population(pop.members[candidate_best_idx])
        else


            testdatasize=12
            #keys_list = collect(keys(traj_dict))
            #random_keys = rand(keys_list, testdatasize)
    
            #randomly select 15 trajectories from each dataset
            keys_list=[]
            for datasetID in keys(trainingID)
                if datasetID != "36_2"
                    testdatasize = 12
                else
                    testdatasize = 25
                end

                singleRandomKeys = StatsBase.sample(trainingID[datasetID],testdatasize, replace=false)
                for item in singleRandomKeys
                    push!(keys_list, item)
                end
            end
            
            traj_dict = Dict()
            for vehID in keys_list
                traj_dict[vehID] = deepcopy(traj_dict_total[vehID])
            end

            #keep only selected trajectories only
            #delvehList=[]
            # Iterate over keys in traj_dict
            #for vehID in keys(traj_dict)
            #    if vehID ∉ keys_list
            #        push!(delvehList, vehID)
            #    end
            #end
    
            # Delete keys from traj_dict that are in delvehList
            #for vehID in delvehList
            #    delete!(traj_dict, vehID)
            #end
            
            #uncomment above for combining CR matrix
            
            best_candidate=Dict()
            GTfunctions = pyimport("PySRFunctions")#pyimport("PySRFunctions")

            num=0
            for idx in candidate_best_idx
                # Optimize constant value
                num=num+1
                #println("ith member:")
                #println(num)
                single_member = pop.members[idx]

                for key in keys_list
                    singleTestTraj=traj_dict[key]
                    FeatureList=singleTestTraj["featureList"]
                    # Create a matrix to store the features
                    X = zeros(length(FeatureList[1]),length(FeatureList))
                    #y = 
                    
                    # Populate the matrix with the feature list values
                    for i in 1:length(FeatureList)  #trajectory length
                        for j in 1:length(FeatureList[1])  #feature numbers
                            X[j, i] = FeatureList[i][j]
                        end
                    end
                    y=singleTestTraj["latdev"]
                    SingleDataset = construct_datasets_simple(X,y)
                    #println(SingleDataset[1].X)
                    #println(SingleDataset[1].X)
                    #println("original dataset")
                    #println(dataset.X)
                    
                    prediction=[]
                    # Evaluate the member's tree
                    (prediction, completion) = eval_tree_array(single_member.tree, SingleDataset[1].X, options)
                    #println(prediction)
                    #(prediction, completion) = eval_tree_dispatch(single_member.tree, SingleDataset[1].X, options, nothing)
                    #print(completion)
                    #traj_dict[key]["latdev"]=[]
 
                    
                    traj_dict[key]["latdev"]=prediction
        
                end
                #println(keys(traj_dict))

                avgCR = GTfunctions.parallel_calculate_CR_matrix(traj_dict)
                #println("avgCR:",avgCR)
                #println("avgLDT:",avgLDT)
                #println("sum:", avgCR+0.1*avgLDT)
                #avgLDT = GTfunctions.parallel_calculate_CR_matrix(traj_dict)
                #compute_complexity(single_member.tree, options)
    
                best_candidate[idx]=avgCR #avgCR+0.1*avgLDT
                
                # Optionally, perform some operations with prediction and completion
                
                # Here we assume some kind of safety check could be performed and add idx to best_idx_safety if it passes
                # For simplicity, we add all indices in this example
                
            
            end
            
            # Sort the dictionary by values in ascending order
            #println("best candidate")
            best_candidate_idx = sort(collect(keys(best_candidate)), by = key -> best_candidate[key])
            GC.gc()
            
            return Population(pop.members[best_candidate_idx[1:10]])
            
        end
    end
    
end




function record_population(pop::Population, options::AbstractOptions)::RecordType
    return RecordType(
        "population" => [
            RecordType(
                "tree" => string_tree(member.tree, options; pretty=false),
                "loss" => member.loss,
                "score" => member.score,
                "complexity" => compute_complexity(member, options),
                "birth" => member.birth,
                "ref" => member.ref,
                "parent" => member.parent,
            ) for member in pop.members
        ],
        "time" => time(),
    )
end

end
